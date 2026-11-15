#include "SmpRig.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <map>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace SmpPhysics
{
	namespace
	{
		constexpr std::size_t kMaxInputBytes = 4u * 1024u * 1024u;
		constexpr int kMaxNestingDepth = 32;
		constexpr std::size_t kMaxFloatTextLength = 64;
		constexpr std::size_t kMaxElementCount = 20000;
		constexpr std::size_t kMaxEntityLength = 8;
		constexpr std::size_t kMaxBones = 512;
		constexpr std::size_t kMaxJoints = 512;
		constexpr float kIdentityTolerance = 1e-6f;
		constexpr float kGravityFactorMinimum = 0.0f;
		constexpr float kGravityFactorMaximum = 1.0f;

		bool IsSpace(char a_character) noexcept
		{
			return a_character == ' ' || a_character == '\t' || a_character == '\n' || a_character == '\r';
		}

		bool IsNameStart(char a_character) noexcept
		{
			return (a_character >= 'a' && a_character <= 'z') || (a_character >= 'A' && a_character <= 'Z') ||
			       a_character == '_' || a_character == ':';
		}

		bool IsNameCharacter(char a_character) noexcept
		{
			return IsNameStart(a_character) || (a_character >= '0' && a_character <= '9') ||
			       a_character == '-' || a_character == '.';
		}

		std::string_view Trimmed(std::string_view a_text) noexcept
		{
			std::size_t begin = 0;
			std::size_t end = a_text.size();
			while (begin < end && IsSpace(a_text[begin]))
				++begin;
			while (end > begin && IsSpace(a_text[end - 1]))
				--end;
			return a_text.substr(begin, end - begin);
		}

		char DecodeEntityName(std::string_view a_name) noexcept
		{
			if (a_name == "amp")
				return '&';
			if (a_name == "lt")
				return '<';
			if (a_name == "gt")
				return '>';
			if (a_name == "quot")
				return '"';
			if (a_name == "apos")
				return '\'';
			return '\0';
		}

		std::string DecodeEntities(std::string_view a_text)
		{
			std::string decoded;
			decoded.reserve(a_text.size());
			for (std::size_t index = 0; index < a_text.size(); ++index) {
				if (a_text[index] != '&') {
					decoded.push_back(a_text[index]);
					continue;
				}
				const std::size_t semicolon = a_text.substr(0, index + kMaxEntityLength + 1).find(';', index + 1);
				if (semicolon == std::string_view::npos || semicolon - index > kMaxEntityLength) {
					decoded.push_back('&');
					continue;
				}
				const char replacement = DecodeEntityName(a_text.substr(index + 1, semicolon - index - 1));
				if (replacement == '\0') {
					decoded.push_back('&');
					continue;
				}
				decoded.push_back(replacement);
				index = semicolon;
			}
			return decoded;
		}

		bool ParseFloatText(std::string_view a_text, float& a_value) noexcept
		{
			std::string_view text = Trimmed(a_text);
			if (!text.empty() && text.front() == '+')
				text.remove_prefix(1);
			if (text.empty())
				return false;
			// hdtSMP64 reads the first decimal comma as a point, and physics files from comma-locale authors rely on it.
			std::array<char, kMaxFloatTextLength> buffer{};
			if (text.size() > buffer.size())
				return false;
			std::copy(text.begin(), text.end(), buffer.begin());
			if (const auto comma = std::find(buffer.begin(), buffer.begin() + text.size(), ','); comma != buffer.begin() + text.size())
				*comma = '.';
			const char* const begin = buffer.data();
			const char* const end = begin + text.size();
			float parsed = 0.0f;
			const std::from_chars_result result = std::from_chars(begin, end, parsed);
			if (result.ec != std::errc() || result.ptr != end || !std::isfinite(parsed))
				return false;
			a_value = parsed;
			return true;
		}

		bool ParseBoolText(std::string_view a_text, bool& a_value) noexcept
		{
			const std::string_view text = Trimmed(a_text);
			if (text == "true") {
				a_value = true;
				return true;
			}
			if (text == "false") {
				a_value = false;
				return true;
			}
			return false;
		}

		struct XmlAttribute
		{
			std::string name;
			std::string value;
		};

		struct XmlElement
		{
			std::string name;
			std::vector<XmlAttribute> attributes;
			std::vector<XmlElement> children;
			std::string text;
		};

		const XmlAttribute* FindAttribute(const XmlElement& a_element, std::string_view a_name) noexcept
		{
			for (const XmlAttribute& attribute : a_element.attributes) {
				if (attribute.name == a_name)
					return &attribute;
			}
			return nullptr;
		}

		std::string AttributeText(const XmlElement& a_element, std::string_view a_name)
		{
			const XmlAttribute* attribute = FindAttribute(a_element, a_name);
			return attribute != nullptr ? attribute->value : std::string();
		}

		struct XmlReader
		{
			std::string_view input;
			std::size_t position{};
			std::size_t elementCount{};
			std::string error;

			bool Fail(std::string a_reason)
			{
				if (error.empty())
					error = std::move(a_reason);
				return false;
			}

			bool AtEnd() const noexcept { return position >= input.size(); }
			char Peek() const noexcept { return position < input.size() ? input[position] : '\0'; }

			bool StartsWith(std::string_view a_text) const noexcept
			{
				return input.size() - position >= a_text.size() &&
				       input.compare(position, a_text.size(), a_text) == 0;
			}

			void SkipWhitespace() noexcept
			{
				while (!AtEnd() && IsSpace(input[position]))
					++position;
			}

			bool SkipComment()
			{
				const std::size_t end = input.find("-->", position + 4);
				if (end == std::string_view::npos)
					return Fail("unterminated comment");
				position = end + 3;
				return true;
			}

			bool SkipMisc()
			{
				for (;;) {
					SkipWhitespace();
					if (!StartsWith("<!--"))
						return true;
					if (!SkipComment())
						return false;
				}
			}

			bool ParseName(std::string& a_name)
			{
				if (AtEnd() || !IsNameStart(Peek()))
					return Fail("expected a name");
				const std::size_t start = position;
				++position;
				while (!AtEnd() && IsNameCharacter(Peek()))
					++position;
				a_name = std::string(input.substr(start, position - start));
				return true;
			}

			bool ParseAttributes(std::vector<XmlAttribute>& a_attributes)
			{
				for (;;) {
					SkipWhitespace();
					if (AtEnd())
						return Fail("unterminated start tag");
					if (Peek() == '>' || Peek() == '/')
						return true;
					XmlAttribute attribute;
					if (!ParseName(attribute.name))
						return false;
					SkipWhitespace();
					if (AtEnd() || Peek() != '=')
						return Fail("attribute '" + attribute.name + "' has no value");
					++position;
					SkipWhitespace();
					if (AtEnd() || (Peek() != '"' && Peek() != '\''))
						return Fail("unterminated attribute '" + attribute.name + "'");
					const char quote = Peek();
					++position;
					const std::size_t valueStart = position;
					while (!AtEnd() && Peek() != quote)
						++position;
					if (AtEnd())
						return Fail("unterminated attribute '" + attribute.name + "'");
					attribute.value = DecodeEntities(input.substr(valueStart, position - valueStart));
					++position;
					a_attributes.push_back(std::move(attribute));
				}
			}

			bool ParseElement(XmlElement& a_element, int a_depth)
			{
				if (a_depth > kMaxNestingDepth)
					return Fail("nesting depth above the limit");
				if (++elementCount > kMaxElementCount)
					return Fail("element count above the limit");
				if (AtEnd() || Peek() != '<')
					return Fail("expected an element");
				++position;
				if (StartsWith("!DOCTYPE"))
					return Fail("DOCTYPE declaration is not supported");
				if (Peek() == '!' || Peek() == '?')
					return Fail("unsupported markup outside an element");
				if (!ParseName(a_element.name))
					return false;
				if (!ParseAttributes(a_element.attributes))
					return false;
				if (AtEnd())
					return Fail("unterminated element '" + a_element.name + "'");
				if (Peek() == '/') {
					++position;
					if (AtEnd() || Peek() != '>')
						return Fail("malformed empty element '" + a_element.name + "'");
					++position;
					return true;
				}
				++position;
				std::string text;
				for (;;) {
					if (AtEnd())
						return Fail("unterminated element '" + a_element.name + "'");
					if (StartsWith("</")) {
						position += 2;
						std::string closingName;
						if (!ParseName(closingName))
							return false;
						SkipWhitespace();
						if (AtEnd() || Peek() != '>')
							return Fail("unterminated closing tag '" + closingName + "'");
						++position;
						if (closingName != a_element.name)
							return Fail("closing tag '" + closingName + "' does not match '" + a_element.name + "'");
						a_element.text = std::string(Trimmed(text));
						return true;
					}
					if (StartsWith("<!--")) {
						if (!SkipComment())
							return false;
						continue;
					}
					if (Peek() == '<') {
						if (StartsWith("<!") || StartsWith("<?"))
							return Fail("unsupported markup in element '" + a_element.name + "'");
						a_element.children.emplace_back();
						if (!ParseElement(a_element.children.back(), a_depth + 1))
							return false;
						continue;
					}
					const std::size_t textStart = position;
					while (!AtEnd() && Peek() != '<')
						++position;
					text += DecodeEntities(input.substr(textStart, position - textStart));
				}
			}

			bool ParseDocument(XmlElement& a_root)
			{
				if (input.size() > kMaxInputBytes)
					return Fail("input size above the limit");
				SkipWhitespace();
				if (StartsWith("<?")) {
					const bool isProlog = StartsWith("<?xml") && position + 5 < input.size() &&
					                      (IsSpace(input[position + 5]) || input[position + 5] == '?');
					if (!isProlog)
						return Fail("processing instruction other than the XML prolog");
					const std::size_t end = input.find("?>", position);
					if (end == std::string_view::npos)
						return Fail("unterminated XML prolog");
					position = end + 2;
				}
				if (!SkipMisc())
					return false;
				if (!ParseElement(a_root, 1))
					return false;
				if (!SkipMisc())
					return false;
				if (!AtEnd())
					return Fail("content after the root element");
				return true;
			}
		};

		bool IsIdentity(const Transform& a_transform) noexcept
		{
			if (std::abs(a_transform.rotation.w) < 1.0f - kIdentityTolerance)
				return false;
			return std::abs(a_transform.translation.x) <= kIdentityTolerance &&
			       std::abs(a_transform.translation.y) <= kIdentityTolerance &&
			       std::abs(a_transform.translation.z) <= kIdentityTolerance;
		}

		/** @brief Where RepairEmptyNumber records what it repaired; set for the duration of one ParseSmpXml call. */
		thread_local std::vector<std::string>* g_parseWarnings = nullptr;

		/**
		 * @brief Whether a number is empty and so keeps its default.
		 * @details hdtSMP64 throws on an empty number and loses the whole file; a blank value is an authoring slip, so the
		 *          value keeps what the template or default set and the parse reports it.
		 */
		bool RepairEmptyNumber(std::string_view a_text, const std::string& a_what)
		{
			if (!Trimmed(a_text).empty())
				return false;
			if (g_parseWarnings != nullptr)
				g_parseWarnings->push_back(a_what + " is empty and keeps its default");
			return true;
		}

		bool ReadFloat(const XmlElement& a_element, float& a_value, std::string& a_reason)
		{
			if (RepairEmptyNumber(a_element.text, "element '" + a_element.name + "'"))
				return true;
			if (!ParseFloatText(a_element.text, a_value)) {
				a_reason = "element '" + a_element.name + "' is not a finite number";
				return false;
			}
			return true;
		}

		bool ReadBool(const XmlElement& a_element, bool& a_value, std::string& a_reason)
		{
			if (!ParseBoolText(a_element.text, a_value)) {
				a_reason = "element '" + a_element.name + "' is not true or false";
				return false;
			}
			return true;
		}

		bool ReadVec3(const XmlElement& a_element, Vec3& a_value, std::string& a_reason)
		{
			constexpr const char* kAxisNames[3] = { "x", "y", "z" };
			float components[3] = { a_value.x, a_value.y, a_value.z };
			for (int axis = 0; axis < 3; ++axis) {
				const XmlAttribute* attribute = FindAttribute(a_element, kAxisNames[axis]);
				if (attribute == nullptr) {
					a_reason = "element '" + a_element.name + "' has no '" + kAxisNames[axis] + "' attribute";
					return false;
				}
				// An empty axis is read as 0, as hdtSMP64 does, not as the default, which for a joint's linear limits is free.
				if (Trimmed(attribute->value).empty()) {
					if (g_parseWarnings != nullptr)
						g_parseWarnings->push_back(std::string("attribute '") + kAxisNames[axis] + "' of '" + a_element.name + "' is empty and reads as 0");
					components[axis] = 0.0f;
					continue;
				}
				if (!ParseFloatText(attribute->value, components[axis])) {
					a_reason = std::string("attribute '") + kAxisNames[axis] + "' of '" + a_element.name + "' is not a finite number";
					return false;
				}
			}
			a_value = { components[0], components[1], components[2] };
			return true;
		}

		enum class ReadStatus
		{
			Ok,
			Malformed,
			Unsupported
		};

		/** @brief Reads a frameInLerp, whose two children are blend factors rather than a transform. */
		bool ReadFrameLerp(const XmlElement& a_element, float& a_translationLerp, float& a_rotationLerp, std::string& a_reason)
		{
			for (const XmlElement& child : a_element.children) {
				if (child.name == "translationLerp") {
					if (!ReadFloat(child, a_translationLerp, a_reason))
						return false;
				} else if (child.name == "rotationLerp") {
					if (!ReadFloat(child, a_rotationLerp, a_reason))
						return false;
				} else {
					a_reason = "unsupported element '" + child.name + "' in 'frameInLerp'";
					return false;
				}
			}
			return true;
		}

		ReadStatus ReadTransform(const XmlElement& a_element, Transform& a_value, std::string& a_reason)
		{
			constexpr const char* kComponentNames[4] = { "w", "x", "y", "z" };
			Quat rotation{};
			Vec3 origin{};
			for (const XmlElement& child : a_element.children) {
				if (child.name == "basis") {
					float components[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
					for (int component = 0; component < 4; ++component) {
						const XmlAttribute* attribute = FindAttribute(child, kComponentNames[component]);
						if (attribute != nullptr && !ParseFloatText(attribute->value, components[component])) {
							a_reason = "attribute '" + std::string(kComponentNames[component]) +
							           "' of 'basis' is not a finite number";
							return ReadStatus::Malformed;
						}
					}
					rotation = Quat{ components[0], components[1], components[2], components[3] };
					const float magnitude = std::sqrt(rotation.w * rotation.w + rotation.x * rotation.x +
													  rotation.y * rotation.y + rotation.z * rotation.z);
					if (!(magnitude > kDirectionEpsilon)) {
						a_reason = "element 'basis' is a zero quaternion";
						return ReadStatus::Malformed;
					}
					rotation = rotation.Normalized();
				} else if (child.name == "origin") {
					if (!ReadVec3(child, origin, a_reason))
						return ReadStatus::Malformed;
				} else {
					a_reason = "unsupported element '" + child.name + "' in '" + a_element.name + "'";
					return ReadStatus::Unsupported;
				}
			}
			a_value = { rotation, origin };
			return ReadStatus::Ok;
		}

		struct BoneTemplate
		{
			float mass{};
			Vec3 inertia{};
			float linearDamping{};
			float angularDamping{};
			float gravityFactor{ 1.0f };
			float windFactor{ 1.0f };
			float friction{ 0.5f };
			float marginMultiplier{ 1.0f };
		};

		struct JointTemplate
		{
			FrameKind frameKind{ FrameKind::FrameInB };
			Transform frame{};
			float translationLerp{};
			float rotationLerp{};
			Vec3 linearLower{ 1.0f, 1.0f, 1.0f };
			Vec3 linearUpper{ -1.0f, -1.0f, -1.0f };
			Vec3 linearStiffness{};
			Vec3 linearDamping{};
			Vec3 linearEquilibrium{};
			Vec3 linearBounce{};
			Vec3 linearNonHookeanStiffness{};
			Vec3 linearNonHookeanDamping{};
			bool linearSpringsEnabled{ true };
			bool linearStiffnessLimited{ true };
			Vec3 angularLower{ 1.0f, 1.0f, 1.0f };
			Vec3 angularUpper{ -1.0f, -1.0f, -1.0f };
			Vec3 angularStiffness{};
			Vec3 angularDamping{};
			Vec3 angularEquilibrium{};
			Vec3 angularBounce{};
			Vec3 angularNonHookeanStiffness{};
			Vec3 angularNonHookeanDamping{};
			bool angularSpringsEnabled{ true };
			bool angularStiffnessLimited{ true };
			bool springDampingLimited{ true };
			bool useLinearReferenceFrameA{};
			bool linearMotors{};
			bool angularMotors{};
			bool linearServoMotors{};
			bool angularServoMotors{};
			Vec3 linearTargetVelocity{};
			Vec3 angularTargetVelocity{};
			Vec3 linearMaxMotorForce{};
			Vec3 angularMaxMotorForce{};
			float stopErp{ 0.2f };
			float motorErp{ 0.9f };
			float motorCfm{};
			std::optional<float> stopCfm;
		};

		struct RigBuilder
		{
			SmpRig rig;
			std::map<std::string, BoneTemplate> boneTemplates;
			std::map<std::string, JointTemplate> jointTemplates;
			std::map<std::string, int> boneIndices;
			std::vector<bool> joinedBones;  ///< per bone, whether a joint already drives it
			std::string error;
			ParseStatus status{ ParseStatus::Ok };

			RigBuilder()
			{
				boneTemplates[std::string()] = BoneTemplate{};
				jointTemplates[std::string()] = JointTemplate{};
			}

			bool Unsupported(std::string a_reason)
			{
				status = ParseStatus::Unsupported;
				error = std::move(a_reason);
				return false;
			}

			bool Malformed(std::string a_reason)
			{
				status = ParseStatus::Malformed;
				error = std::move(a_reason);
				return false;
			}

			/** @brief Turns a transform read outcome into the matching builder status, false for both failure kinds. */
			bool Record(ReadStatus a_readStatus, const std::string& a_reason)
			{
				if (a_readStatus == ReadStatus::Ok)
					return true;
				if (a_readStatus == ReadStatus::Unsupported)
					return Unsupported(a_reason);
				return Malformed(a_reason);
			}
		};

		BoneTemplate GetBoneTemplate(const RigBuilder& a_builder, const std::string& a_name)
		{
			const auto found = a_builder.boneTemplates.find(a_name);
			if (found != a_builder.boneTemplates.end())
				return found->second;
			return a_builder.boneTemplates.at(std::string());
		}

		JointTemplate GetJointTemplate(const RigBuilder& a_builder, const std::string& a_name)
		{
			const auto found = a_builder.jointTemplates.find(a_name);
			if (found != a_builder.jointTemplates.end())
				return found->second;
			return a_builder.jointTemplates.at(std::string());
		}

		bool ApplyBoneChildren(RigBuilder& a_builder, BoneTemplate& a_template, const XmlElement& a_element,
			const std::string& a_context)
		{
			std::string reason;
			for (const XmlElement& child : a_element.children) {
				const std::string& name = child.name;
				float ignored = 0.0f;
				if (name == "mass") {
					if (!ReadFloat(child, a_template.mass, reason))
						return a_builder.Malformed(reason);
				} else if (name == "inertia") {
					if (!ReadVec3(child, a_template.inertia, reason))
						return a_builder.Malformed(reason);
				} else if (name == "centerOfMassTransform") {
					Transform value;
					const ReadStatus readStatus = ReadTransform(child, value, reason);
					if (!a_builder.Record(readStatus, reason))
						return false;
					if (!IsIdentity(value))
						return a_builder.Unsupported("non-identity centerOfMassTransform in " + a_context);
				} else if (name == "linearDamping") {
					if (!ReadFloat(child, a_template.linearDamping, reason))
						return a_builder.Malformed(reason);
				} else if (name == "angularDamping") {
					if (!ReadFloat(child, a_template.angularDamping, reason))
						return a_builder.Malformed(reason);
				} else if (name == "gravity-factor") {
					float gravityFactor = 0.0f;
					if (!ReadFloat(child, gravityFactor, reason))
						return a_builder.Malformed(reason);
					a_template.gravityFactor = std::clamp(gravityFactor, kGravityFactorMinimum, kGravityFactorMaximum);
				} else if (name == "wind-factor") {
					if (!ReadFloat(child, a_template.windFactor, reason))
						return a_builder.Malformed(reason);
				} else if (name == "friction") {
					if (!ReadFloat(child, a_template.friction, reason))
						return a_builder.Malformed(reason);
				} else if (name == "margin-multiplier") {
					if (!ReadFloat(child, a_template.marginMultiplier, reason))
						return a_builder.Malformed(reason);
				} else if (name == "rollingFriction" || name == "restitution") {
					if (!ReadFloat(child, ignored, reason))
						return a_builder.Malformed(reason);
				} else {
					return a_builder.Unsupported("unsupported element '" + name + "' in " + a_context);
				}
			}
			return true;
		}

		bool ApplyJointChildren(RigBuilder& a_builder, JointTemplate& a_template, const XmlElement& a_element)
		{
			std::string reason;
			for (const XmlElement& child : a_element.children) {
				const std::string& name = child.name;
				if (name == "frameInA" || name == "frameInB") {
					Transform value;
					const ReadStatus readStatus = ReadTransform(child, value, reason);
					if (!a_builder.Record(readStatus, reason))
						return false;
					a_template.frameKind = name == "frameInA" ? FrameKind::FrameInA : FrameKind::FrameInB;
					a_template.frame = value;
				} else if (name == "frameInLerp") {
					if (!ReadFrameLerp(child, a_template.translationLerp, a_template.rotationLerp, reason))
						return a_builder.Malformed(reason);
					a_template.frameKind = FrameKind::FrameInLerp;
					a_template.frame = Transform{};
				} else if (name == "useLinearReferenceFrameA") {
					if (!ReadBool(child, a_template.useLinearReferenceFrameA, reason))
						return a_builder.Malformed(reason);
				} else if (name == "linearLowerLimit") {
					if (!ReadVec3(child, a_template.linearLower, reason))
						return a_builder.Malformed(reason);
				} else if (name == "linearUpperLimit") {
					if (!ReadVec3(child, a_template.linearUpper, reason))
						return a_builder.Malformed(reason);
				} else if (name == "angularLowerLimit") {
					if (!ReadVec3(child, a_template.angularLower, reason))
						return a_builder.Malformed(reason);
				} else if (name == "angularUpperLimit") {
					if (!ReadVec3(child, a_template.angularUpper, reason))
						return a_builder.Malformed(reason);
				} else if (name == "angularStiffness") {
					if (!ReadVec3(child, a_template.angularStiffness, reason))
						return a_builder.Malformed(reason);
				} else if (name == "angularDamping") {
					if (!ReadVec3(child, a_template.angularDamping, reason))
						return a_builder.Malformed(reason);
				} else if (name == "angularEquilibrium") {
					if (!ReadVec3(child, a_template.angularEquilibrium, reason))
						return a_builder.Malformed(reason);
				} else if (name == "enableAngularSprings") {
					if (!ReadBool(child, a_template.angularSpringsEnabled, reason))
						return a_builder.Malformed(reason);
				} else if (name == "linearNonHookeanDamping") {
					if (!ReadVec3(child, a_template.linearNonHookeanDamping, reason))
						return a_builder.Malformed(reason);
				} else if (name == "linearNonHookeanStiffness") {
					if (!ReadVec3(child, a_template.linearNonHookeanStiffness, reason))
						return a_builder.Malformed(reason);
				} else if (name == "angularNonHookeanDamping") {
					if (!ReadVec3(child, a_template.angularNonHookeanDamping, reason))
						return a_builder.Malformed(reason);
				} else if (name == "angularNonHookeanStiffness") {
					if (!ReadVec3(child, a_template.angularNonHookeanStiffness, reason))
						return a_builder.Malformed(reason);
				} else if (name == "linearStiffness") {
					if (!ReadVec3(child, a_template.linearStiffness, reason))
						return a_builder.Malformed(reason);
				} else if (name == "linearDamping") {
					if (!ReadVec3(child, a_template.linearDamping, reason))
						return a_builder.Malformed(reason);
				} else if (name == "linearEquilibrium") {
					if (!ReadVec3(child, a_template.linearEquilibrium, reason))
						return a_builder.Malformed(reason);
				} else if (name == "linearBounce") {
					if (!ReadVec3(child, a_template.linearBounce, reason))
						return a_builder.Malformed(reason);
				} else if (name == "angularBounce") {
					if (!ReadVec3(child, a_template.angularBounce, reason))
						return a_builder.Malformed(reason);
				} else if (name == "angularStiffnessLimited") {
					if (!ReadBool(child, a_template.angularStiffnessLimited, reason))
						return a_builder.Malformed(reason);
				} else if (name == "springDampingLimited") {
					if (!ReadBool(child, a_template.springDampingLimited, reason))
						return a_builder.Malformed(reason);
				} else if (name == "enableLinearSprings") {
					if (!ReadBool(child, a_template.linearSpringsEnabled, reason))
						return a_builder.Malformed(reason);
				} else if (name == "linearStiffnessLimited") {
					if (!ReadBool(child, a_template.linearStiffnessLimited, reason))
						return a_builder.Malformed(reason);
				} else if (name == "linearMotors") {
					if (!ReadBool(child, a_template.linearMotors, reason))
						return a_builder.Malformed(reason);
				} else if (name == "angularMotors") {
					if (!ReadBool(child, a_template.angularMotors, reason))
						return a_builder.Malformed(reason);
				} else if (name == "linearServoMotors") {
					if (!ReadBool(child, a_template.linearServoMotors, reason))
						return a_builder.Malformed(reason);
				} else if (name == "angularServoMotors") {
					if (!ReadBool(child, a_template.angularServoMotors, reason))
						return a_builder.Malformed(reason);
				} else if (name == "linearTargetVelocity") {
					if (!ReadVec3(child, a_template.linearTargetVelocity, reason))
						return a_builder.Malformed(reason);
				} else if (name == "angularTargetVelocity") {
					if (!ReadVec3(child, a_template.angularTargetVelocity, reason))
						return a_builder.Malformed(reason);
				} else if (name == "linearMaxMotorForce") {
					if (!ReadVec3(child, a_template.linearMaxMotorForce, reason))
						return a_builder.Malformed(reason);
				} else if (name == "angularMaxMotorForce") {
					if (!ReadVec3(child, a_template.angularMaxMotorForce, reason))
						return a_builder.Malformed(reason);
				} else if (name == "stopERP") {
					if (!ReadFloat(child, a_template.stopErp, reason))
						return a_builder.Malformed(reason);
				} else if (name == "stopCFM") {
					float stopCfm = 0.0f;
					if (!ReadFloat(child, stopCfm, reason))
						return a_builder.Malformed(reason);
					a_template.stopCfm = stopCfm;
				} else if (name == "motorERP") {
					if (!ReadFloat(child, a_template.motorErp, reason))
						return a_builder.Malformed(reason);
				} else if (name == "motorCFM") {
					if (!ReadFloat(child, a_template.motorCfm, reason))
						return a_builder.Malformed(reason);
				} else {
					return a_builder.Unsupported("unsupported element '" + name + "' in a generic-constraint");
				}
			}
			return true;
		}

		/** @brief Reads one collision shape. Every child that reaches the contact model is recorded; the rest are accepted and dropped. */
		bool ApplyShapeChildren(RigBuilder& a_builder, RigShape& a_shape, const XmlElement& a_element, bool a_perVertex)
		{
			std::string reason;
			for (const XmlElement& child : a_element.children) {
				const std::string& name = child.name;
				if (name == "margin") {
					if (!ReadFloat(child, a_shape.margin, reason))
						return a_builder.Malformed(reason);
				} else if (name == "prenetration" || name == "penetration") {
					if (!ReadFloat(child, a_shape.penetration, reason))
						return a_builder.Malformed(reason);
				} else if (name == "shared") {
					a_shape.shared = child.text;
				} else if (name == "tag") {
					a_shape.tag = child.text;
				} else if (name == "priority") {
					float priority = 0.0f;
					if (!ReadFloat(child, priority, reason))
						return a_builder.Malformed(reason);
					a_shape.priority = static_cast<int>(priority);
				} else if (name == "can-collide-with-tag") {
					a_shape.collidesWith.push_back(child.text);
				} else if (name == "no-collide-with-tag") {
					a_shape.noCollidesWith.push_back(child.text);
				} else if (name == "no-collide-with-bone") {
					a_shape.noCollidesWithBones.push_back(child.text);
				} else if (name == "weight-threshold") {
					const XmlAttribute* bone = FindAttribute(child, "bone");
					if (bone == nullptr)
						return a_builder.Malformed("element 'weight-threshold' in a shape has no bone");
					RigShapeWeight weight;
					weight.bone = bone->value;
					if (!ReadFloat(child, weight.threshold, reason))
						return a_builder.Malformed(reason);
					a_shape.weights.push_back(std::move(weight));
				} else if (name == "disable-tag") {
				} else {
					return a_builder.Unsupported(
						"unsupported element '" + name + "' in " + (a_perVertex ? "a per-vertex-shape" : "a per-triangle-shape"));
				}
			}
			return true;
		}

		bool ProcessTopLevel(RigBuilder& a_builder, const XmlElement& a_parent)
		{
			for (const XmlElement& element : a_parent.children) {
				const std::string& name = element.name;
				if (name == "bone-default") {
					const std::string key = AttributeText(element, "name");
					BoneTemplate working = GetBoneTemplate(a_builder, AttributeText(element, "extends"));
					if (!ApplyBoneChildren(a_builder, working, element, "bone-default '" + key + "'"))
						return false;
					a_builder.boneTemplates[key] = working;
				} else if (name == "bone") {
					const XmlAttribute* nameAttribute = FindAttribute(element, "name");
					if (nameAttribute == nullptr)
						return a_builder.Malformed("element 'bone' has no name");
					if (a_builder.boneIndices.find(nameAttribute->value) != a_builder.boneIndices.end())
						continue;
					BoneTemplate working = GetBoneTemplate(a_builder, AttributeText(element, "template"));
					if (!ApplyBoneChildren(a_builder, working, element, "bone '" + nameAttribute->value + "'"))
						return false;
					RigBone bone;
					bone.name = nameAttribute->value;
					bone.kinematic = working.mass == 0.0f;
					bone.mass = working.mass;
					bone.inertia = working.inertia;
					bone.angularDamping = working.angularDamping;
					bone.linearDamping = working.linearDamping;
					bone.gravityFactor = working.gravityFactor;
					bone.windFactor = working.windFactor;
					bone.friction = working.friction;
					bone.marginMultiplier = working.marginMultiplier;
					a_builder.boneIndices[bone.name] = static_cast<int>(a_builder.rig.bones.size());
					a_builder.rig.bones.push_back(std::move(bone));
					a_builder.joinedBones.push_back(false);
				} else if (name == "generic-constraint-default") {
					const std::string key = AttributeText(element, "name");
					JointTemplate working = GetJointTemplate(a_builder, AttributeText(element, "extends"));
					if (!ApplyJointChildren(a_builder, working, element))
						return false;
					a_builder.jointTemplates[key] = working;
				} else if (name == "generic-constraint") {
					const XmlAttribute* bodyA = FindAttribute(element, "bodyA");
					const XmlAttribute* bodyB = FindAttribute(element, "bodyB");
					if (bodyA == nullptr || bodyB == nullptr)
						return a_builder.Malformed("element 'generic-constraint' needs bodyA and bodyB");
					JointTemplate working = GetJointTemplate(a_builder, AttributeText(element, "template"));
					if (!ApplyJointChildren(a_builder, working, element))
						return false;
					const auto child = a_builder.boneIndices.find(bodyA->value);
					const auto parent = a_builder.boneIndices.find(bodyB->value);
					if (child == a_builder.boneIndices.end() || parent == a_builder.boneIndices.end())
						continue;
					if (child->second == parent->second) {
						if (g_parseWarnings != nullptr)
							g_parseWarnings->push_back("generic-constraint joins bone '" + bodyA->value + "' to itself and is dropped");
						continue;
					}
					RigJoint joint;
					joint.child = child->second;
					joint.parent = parent->second;
					joint.frameKind = working.frameKind;
					joint.frame = working.frame;
					joint.translationLerp = working.translationLerp;
					joint.rotationLerp = working.rotationLerp;
					joint.linearLower = working.linearLower;
					joint.linearUpper = working.linearUpper;
					joint.linearStiffness = working.linearStiffness;
					joint.linearDamping = working.linearDamping;
					joint.linearEquilibrium = working.linearEquilibrium;
					joint.linearBounce = working.linearBounce;
					joint.linearNonHookeanStiffness = working.linearNonHookeanStiffness;
					joint.linearNonHookeanDamping = working.linearNonHookeanDamping;
					joint.linearSpringsEnabled = working.linearSpringsEnabled;
					joint.linearStiffnessLimited = working.linearStiffnessLimited;
					joint.angularLower = working.angularLower;
					joint.angularUpper = working.angularUpper;
					joint.angularStiffness = working.angularStiffness;
					joint.angularDamping = working.angularDamping;
					joint.angularEquilibrium = working.angularEquilibrium;
					joint.angularBounce = working.angularBounce;
					joint.angularNonHookeanStiffness = working.angularNonHookeanStiffness;
					joint.angularNonHookeanDamping = working.angularNonHookeanDamping;
					joint.angularSpringsEnabled = working.angularSpringsEnabled;
					joint.angularStiffnessLimited = working.angularStiffnessLimited;
					joint.springDampingLimited = working.springDampingLimited;
					joint.useLinearReferenceFrameA = working.useLinearReferenceFrameA;
					joint.linearMotors = working.linearMotors;
					joint.angularMotors = working.angularMotors;
					joint.linearServoMotors = working.linearServoMotors;
					joint.angularServoMotors = working.angularServoMotors;
					joint.linearTargetVelocity = working.linearTargetVelocity;
					joint.angularTargetVelocity = working.angularTargetVelocity;
					joint.linearMaxMotorForce = working.linearMaxMotorForce;
					joint.angularMaxMotorForce = working.angularMaxMotorForce;
					joint.stopErp = working.stopErp;
					joint.motorErp = working.motorErp;
					joint.motorCfm = working.motorCfm;
					joint.stopCfm = working.stopCfm;
					if (a_builder.joinedBones[static_cast<std::size_t>(joint.child)])
						a_builder.rig.additionalJoints.push_back(joint);
					else {
						a_builder.joinedBones[static_cast<std::size_t>(joint.child)] = true;
						a_builder.rig.joints.push_back(joint);
					}
				} else if (name == "per-vertex-shape" || name == "per-triangle-shape") {
					const XmlAttribute* shapeName = FindAttribute(element, "name");
					if (shapeName == nullptr)
						return a_builder.Malformed("element '" + name + "' has no name");
					RigShape shape;
					shape.name = Trimmed(shapeName->value);
					const bool perVertex = name == "per-vertex-shape";
					if (!ApplyShapeChildren(a_builder, shape, element, perVertex))
						return false;
					if (perVertex)
						a_builder.rig.collision.vertexShapes.push_back(std::move(shape));
					else
						a_builder.rig.collision.triangleShapes.push_back(std::move(shape));
				} else if (name == "constraint-group") {
					if (!ProcessTopLevel(a_builder, element))
						return false;
				} else if (name == "shape" || name == "stiffspring-constraint" || name == "stiffspring-constraint-default" ||
						   name == "conetwist-constraint" || name == "conetwist-constraint-default") {
					return a_builder.Unsupported("unsupported top-level element '" + name + "'");
				}
			}
			return true;
		}
	}

	JointFrames ResolveJointFrames(const RigJoint& a_joint, const Transform& a_childRest, const Transform& a_parentRest)
	{
		JointFrames frames;
		if (a_joint.frameKind == FrameKind::FrameInLerp) {
			const Transform blend{ Slerp(a_childRest.rotation, a_parentRest.rotation, a_joint.rotationLerp),
				a_childRest.translation + (a_parentRest.translation - a_childRest.translation) * a_joint.translationLerp };
			frames.frameA = a_childRest.Inverse() * blend;
			frames.frameB = a_parentRest.Inverse() * blend;
		} else if (a_joint.frameKind == FrameKind::FrameInA) {
			frames.frameA = a_joint.frame;
			frames.frameB = a_parentRest.Inverse() * (a_childRest * frames.frameA);
		} else {
			frames.frameB = a_joint.frame;
			frames.frameA = a_childRest.Inverse() * (a_parentRest * frames.frameB);
		}
		return frames;
	}

	bool ValidateRig(const SmpRig& a_rig, std::string& a_reason, bool a_requireForest)
	{
		if (a_rig.bones.size() > kMaxBones) {
			a_reason = "bone count above the limit";
			return false;
		}
		if (a_rig.joints.size() > kMaxJoints) {
			a_reason = "joint count above the limit";
			return false;
		}
		for (const RigBone& bone : a_rig.bones) {
			if (!std::isfinite(bone.mass) || bone.mass < 0.0f) {
				a_reason = "bone '" + bone.name + "' has a non-finite or negative mass";
				return false;
			}
		}
		std::vector<int> jointOfBone(a_rig.bones.size(), -1);
		for (std::size_t index = 0; index < a_rig.joints.size(); ++index) {
			const RigJoint& joint = a_rig.joints[index];
			if (joint.child < 0 || static_cast<std::size_t>(joint.child) >= a_rig.bones.size() ||
				joint.parent < 0 || static_cast<std::size_t>(joint.parent) >= a_rig.bones.size()) {
				a_reason = "joint " + std::to_string(index) + " references a bone outside the rig";
				return false;
			}
			const RigBone& childBone = a_rig.bones[static_cast<std::size_t>(joint.child)];
			if (childBone.kinematic) {
				a_reason = "joint " + std::to_string(index) + " has the kinematic bone '" + childBone.name +
				           "' as its child";
				return false;
			}
			if (jointOfBone[static_cast<std::size_t>(joint.child)] >= 0) {
				a_reason = "bone '" + childBone.name + "' is the child of more than one joint";
				return false;
			}
			jointOfBone[static_cast<std::size_t>(joint.child)] = static_cast<int>(index);
		}
		if (!a_requireForest)
			return true;
		for (std::size_t index = 0; index < a_rig.bones.size(); ++index) {
			if (!a_rig.bones[index].kinematic && jointOfBone[index] < 0) {
				a_reason = "dynamic bone '" + a_rig.bones[index].name + "' has no joint";
				return false;
			}
		}
		for (std::size_t index = 0; index < a_rig.bones.size(); ++index) {
			if (a_rig.bones[index].kinematic)
				continue;
			int current = static_cast<int>(index);
			std::size_t steps = 0;
			while (current >= 0 && !a_rig.bones[static_cast<std::size_t>(current)].kinematic) {
				if (++steps > a_rig.bones.size()) {
					a_reason = "joint chain starting at bone '" + a_rig.bones[index].name + "' is cyclic";
					return false;
				}
				current = a_rig.joints[static_cast<std::size_t>(jointOfBone[static_cast<std::size_t>(current)])].parent;
			}
		}
		return true;
	}

	std::vector<int> BuildParentFirstOrder(const SmpRig& a_rig, const std::vector<int>& a_jointOfBone)
	{
		std::vector<int> order;
		std::vector<int> depths(a_rig.bones.size(), 0);
		for (std::size_t index = 0; index < a_rig.bones.size(); ++index) {
			if (a_rig.bones[index].kinematic)
				continue;
			int depth = 0;
			int current = static_cast<int>(index);
			while (current >= 0 && !a_rig.bones[static_cast<std::size_t>(current)].kinematic) {
				++depth;
				if (static_cast<std::size_t>(depth) > a_rig.bones.size())
					break;
				current = a_rig.joints[static_cast<std::size_t>(a_jointOfBone[static_cast<std::size_t>(current)])].parent;
			}
			depths[index] = depth;
			order.push_back(static_cast<int>(index));
		}
		std::stable_sort(order.begin(), order.end(), [&depths](int a_left, int a_right) {
			return depths[static_cast<std::size_t>(a_left)] < depths[static_cast<std::size_t>(a_right)];
		});
		return order;
	}

	ParseResult ParseSmpXml(std::string_view a_xml)
	{
		ParseResult result;
		XmlReader reader{ a_xml };
		XmlElement root;
		if (!reader.ParseDocument(root)) {
			result.status = ParseStatus::Malformed;
			result.reason = reader.error.empty() ? std::string("malformed XML") : reader.error;
			return result;
		}
		if (root.name != "system") {
			result.status = ParseStatus::Malformed;
			result.reason = "root element is '" + root.name + "', expected 'system'";
			return result;
		}
		RigBuilder builder;
		std::vector<std::string> warnings;
		g_parseWarnings = &warnings;
		const bool processed = ProcessTopLevel(builder, root);
		g_parseWarnings = nullptr;
		if (!processed) {
			result.status = builder.status;
			result.reason = builder.error.empty() ? std::string("unsupported content") : builder.error;
			return result;
		}
		std::string reason;
		if (!ValidateRig(builder.rig, reason, false)) {
			result.status = ParseStatus::Unsupported;
			result.reason = std::move(reason);
			return result;
		}
		result.status = ParseStatus::Ok;
		result.warnings = std::move(warnings);
		result.rig = std::move(builder.rig);
		return result;
	}

	bool AllowsCollisionWithTag(const RigShape& a_shape, const std::string& a_tag)
	{
		if (a_shape.collidesWith.empty())
			return std::ranges::find(a_shape.noCollidesWith, a_tag) == a_shape.noCollidesWith.end();
		return std::ranges::find(a_shape.collidesWith, a_tag) != a_shape.collidesWith.end();
	}

	bool ShapesCanCollide(const RigShape& a_first, const RigShape& a_second)
	{
		return AllowsCollisionWithTag(a_first, a_second.tag) && AllowsCollisionWithTag(a_second, a_first.tag);
	}

	std::uint32_t AllowedSurfaceMask(const RigShape* a_boneShape, const std::vector<const RigShape*>& a_surfaceShapes)
	{
		if (a_boneShape == nullptr)
			return 0xFFFFFFFFu;
		std::uint32_t mask = 0u;
		for (std::size_t index = 0; index < a_surfaceShapes.size() && index < 32u; ++index) {
			const RigShape* surface = a_surfaceShapes[index];
			if (surface == nullptr || (surface->name != a_boneShape->name && ShapesCanCollide(*a_boneShape, *surface)))
				mask |= 1u << index;
		}
		return mask;
	}

	SmpRig DropBones(const SmpRig& a_rig, const std::vector<bool>& a_drop)
	{
		SmpRig kept;
		kept.collision = a_rig.collision;
		std::vector<int> newIndex(a_rig.bones.size(), -1);
		for (std::size_t index = 0; index < a_rig.bones.size(); ++index) {
			if (index < a_drop.size() && a_drop[index])
				continue;
			newIndex[index] = static_cast<int>(kept.bones.size());
			kept.bones.push_back(a_rig.bones[index]);
		}
		const auto remap = [&newIndex](const std::vector<RigJoint>& a_from, std::vector<RigJoint>& a_to) {
			for (RigJoint joint : a_from) {
				const int child = newIndex[static_cast<std::size_t>(joint.child)];
				const int parent = newIndex[static_cast<std::size_t>(joint.parent)];
				if (child < 0 || parent < 0)
					continue;
				joint.child = child;
				joint.parent = parent;
				a_to.push_back(joint);
			}
		};
		remap(a_rig.joints, kept.joints);
		remap(a_rig.additionalJoints, kept.additionalJoints);
		return kept;
	}

	namespace
	{
		constexpr int kAnchored = -1;
		constexpr int kCyclic = -2;

		/** @brief Each bone's chain end: kAnchored when it reaches a kinematic bone, kCyclic on a loop, else the bone that has no joint above it. */
		std::vector<int> FindChainEnds(const SmpRig& a_rig)
		{
			std::vector<int> jointOfBone(a_rig.bones.size(), -1);
			for (std::size_t index = 0; index < a_rig.joints.size(); ++index)
				jointOfBone[static_cast<std::size_t>(a_rig.joints[index].child)] = static_cast<int>(index);
			std::vector<int> ends(a_rig.bones.size(), kAnchored);
			for (std::size_t index = 0; index < a_rig.bones.size(); ++index) {
				int current = static_cast<int>(index);
				std::size_t steps = 0;
				while (!a_rig.bones[static_cast<std::size_t>(current)].kinematic) {
					const int joint = jointOfBone[static_cast<std::size_t>(current)];
					if (joint < 0) {
						ends[index] = current;
						break;
					}
					if (++steps > a_rig.bones.size()) {
						ends[index] = kCyclic;
						break;
					}
					current = a_rig.joints[static_cast<std::size_t>(joint)].parent;
				}
			}
			return ends;
		}

		void ReverseJoint(RigJoint& a_joint)
		{
			std::swap(a_joint.child, a_joint.parent);
			if (a_joint.frameKind == FrameKind::FrameInA)
				a_joint.frameKind = FrameKind::FrameInB;
			else if (a_joint.frameKind == FrameKind::FrameInB)
				a_joint.frameKind = FrameKind::FrameInA;
			else {
				a_joint.translationLerp = 1.0f - a_joint.translationLerp;
				a_joint.rotationLerp = 1.0f - a_joint.rotationLerp;
			}
			a_joint.reversed = !a_joint.reversed;
		}
	}

	bool NormalizeRigTopology(SmpRig& a_rig, std::string& a_reason)
	{
		for (std::size_t round = 0; round <= a_rig.bones.size(); ++round) {
			const std::vector<int> ends = FindChainEnds(a_rig);
			int root = -1;
			for (std::size_t index = 0; index < ends.size(); ++index) {
				if (ends[index] >= 0) {
					root = ends[index];
					break;
				}
			}
			if (root < 0)
				return true;

			const auto inGroup = [&ends, root](int a_bone) { return ends[static_cast<std::size_t>(a_bone)] == root; };
			const auto outsideAnchor = [&ends](int a_bone) { return ends[static_cast<std::size_t>(a_bone)] == kAnchored; };
			int connecting = -1;
			int inside = -1;
			for (std::size_t index = 0; index < a_rig.additionalJoints.size() && connecting < 0; ++index) {
				const RigJoint& joint = a_rig.additionalJoints[index];
				if (inGroup(joint.child) && outsideAnchor(joint.parent)) {
					connecting = static_cast<int>(index);
					inside = joint.child;
				} else if (inGroup(joint.parent) && outsideAnchor(joint.child)) {
					connecting = static_cast<int>(index);
					inside = joint.parent;
				}
			}

			if (connecting < 0) {
				std::vector<bool> drop(a_rig.bones.size(), false);
				for (std::size_t index = 0; index < ends.size(); ++index)
					drop[index] = ends[index] == root;
				a_rig = DropBones(a_rig, drop);
				continue;
			}

			std::vector<int> jointOfBone(a_rig.bones.size(), -1);
			for (std::size_t index = 0; index < a_rig.joints.size(); ++index)
				jointOfBone[static_cast<std::size_t>(a_rig.joints[index].child)] = static_cast<int>(index);
			for (int bone = inside; bone != root;) {
				RigJoint& joint = a_rig.joints[static_cast<std::size_t>(jointOfBone[static_cast<std::size_t>(bone)])];
				bone = joint.parent;
				ReverseJoint(joint);
			}

			RigJoint joint = a_rig.additionalJoints[static_cast<std::size_t>(connecting)];
			a_rig.additionalJoints.erase(a_rig.additionalJoints.begin() + connecting);
			if (joint.child != inside)
				ReverseJoint(joint);
			a_rig.joints.push_back(joint);
		}
		a_reason = "the joints could not be rooted at a kinematic bone";
		return false;
	}

	void ApplySpringLimits(SmpRig& a_rig, float a_step, float a_minimumDampingRatio) noexcept
	{
		if (!(a_step > 0.0f))
			return;
		for (RigJoint& joint : a_rig.joints) {
			const RigBone& child = a_rig.bones[static_cast<std::size_t>(joint.child)];
			const RigBone& parent = a_rig.bones[static_cast<std::size_t>(joint.parent)];
			const float childInertia = (child.inertia.x + child.inertia.y + child.inertia.z) / 3.0f;
			const float parentInertia = (parent.inertia.x + parent.inertia.y + parent.inertia.z) / 3.0f;
			float inverseInertia = childInertia > 0.0f ? 1.0f / childInertia : 0.0f;
			if (!parent.kinematic && parentInertia > 0.0f)
				inverseInertia += 1.0f / parentInertia;
			if (!(inverseInertia > 0.0f))
				continue;
			const float reduced = 1.0f / inverseInertia;
			if (joint.angularStiffnessLimited) {
				const float cap = reduced / (16.0f * a_step * a_step);
				joint.angularStiffness = { std::min(joint.angularStiffness.x, cap), std::min(joint.angularStiffness.y, cap),
					std::min(joint.angularStiffness.z, cap) };
			}
			if (joint.springDampingLimited) {
				const float cap = reduced / a_step;
				joint.angularDamping = { std::min(joint.angularDamping.x, cap), std::min(joint.angularDamping.y, cap),
					std::min(joint.angularDamping.z, cap) };
			}
			if (a_minimumDampingRatio > 0.0f) {
				const auto floorFor = [&](float a_stiffness, float a_damping) {
					return std::max(a_damping, 2.0f * a_minimumDampingRatio * std::sqrt(std::max(a_stiffness, 0.0f) * reduced));
				};
				joint.angularDamping = { floorFor(joint.angularStiffness.x, joint.angularDamping.x),
					floorFor(joint.angularStiffness.y, joint.angularDamping.y), floorFor(joint.angularStiffness.z, joint.angularDamping.z) };
			}
		}
	}
}
