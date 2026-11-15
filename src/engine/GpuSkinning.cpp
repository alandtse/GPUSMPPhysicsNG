#include "engine/GpuSkinning.h"

#include "Buffer.h"
#include "Globals.h"
#include "GpuPass.h"
#include "Utils/D3D.h"
#include "smp/GpuViews.h"

#include <algorithm>
#include <cstring>
#include <exception>
#include <memory>
#include <stdexcept>
#include <utility>

namespace SmpPhysics
{
	namespace
	{
		constexpr const wchar_t* kShaderPath = L"SmpPhysics\\SkinPaletteCS.hlsl";
		constexpr const wchar_t* kBatchShaderPath = L"SmpPhysics\\SkinPaletteBatchCS.hlsl";
		constexpr std::uint32_t kEngineCurrentSlot = 10;
		constexpr std::uint32_t kEnginePreviousSlot = 9;

		/** @brief Creates a named buffer and the views asked for; false on any failure, so the caller can fall back to the CPU path. */
		bool CreateBufferWithViews(const D3D11_BUFFER_DESC& a_desc, const char* a_name, winrt::com_ptr<ID3D11Buffer>& a_buffer,
			const D3D11_SHADER_RESOURCE_VIEW_DESC* a_srvDesc = nullptr, winrt::com_ptr<ID3D11ShaderResourceView>* a_srv = nullptr,
			const D3D11_UNORDERED_ACCESS_VIEW_DESC* a_uavDesc = nullptr, winrt::com_ptr<ID3D11UnorderedAccessView>* a_uav = nullptr)
		{
			try {
				Buffer buffer(a_desc, nullptr, a_name);
				if (a_srvDesc)
					buffer.CreateSRV(*a_srvDesc);
				if (a_uavDesc)
					buffer.CreateUAV(*a_uavDesc);
				a_buffer = buffer.resource;
				if (a_srv)
					*a_srv = buffer.srv;
				if (a_uav)
					*a_uav = buffer.uav;
				return true;
			} catch (const std::exception&) {
				return false;
			}
		}

		bool CreateRawBuffer(bool a_writable, const char* a_name, winrt::com_ptr<ID3D11Buffer>& a_buffer,
			winrt::com_ptr<ID3D11ShaderResourceView>& a_srv, winrt::com_ptr<ID3D11UnorderedAccessView>* a_uav,
			std::uint32_t a_bytes = GpuSkinning::kPaletteBytes)
		{
			D3D11_BUFFER_DESC desc{};
			desc.ByteWidth = a_bytes;
			desc.Usage = D3D11_USAGE_DEFAULT;
			desc.BindFlags = D3D11_BIND_SHADER_RESOURCE | (a_writable ? D3D11_BIND_UNORDERED_ACCESS : 0u);
			desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;

			D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc{};
			srvDesc.Format = DXGI_FORMAT_R32_TYPELESS;
			srvDesc.ViewDimension = D3D11_SRV_DIMENSION_BUFFEREX;
			srvDesc.BufferEx.NumElements = a_bytes / sizeof(std::uint32_t);
			srvDesc.BufferEx.Flags = D3D11_BUFFEREX_SRV_FLAG_RAW;

			D3D11_UNORDERED_ACCESS_VIEW_DESC uavDesc{};
			uavDesc.Format = DXGI_FORMAT_R32_TYPELESS;
			uavDesc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
			uavDesc.Buffer.NumElements = a_bytes / sizeof(std::uint32_t);
			uavDesc.Buffer.Flags = D3D11_BUFFER_UAV_FLAG_RAW;
			return CreateBufferWithViews(desc, a_name, a_buffer, &srvDesc, &a_srv, a_uav ? &uavDesc : nullptr, a_uav);
		}

		bool CreateStructuredBuffer(const D3D11_BUFFER_DESC& a_desc, std::uint32_t a_count, const char* a_name,
			winrt::com_ptr<ID3D11Buffer>& a_buffer, winrt::com_ptr<ID3D11ShaderResourceView>& a_srv)
		{
			const D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc = StructuredSrvDesc(a_count);
			return CreateBufferWithViews(a_desc, a_name, a_buffer, &srvDesc, &a_srv);
		}

		bool CreatePaletteConstants(const char* a_name, winrt::com_ptr<ID3D11Buffer>& a_buffer, std::uint32_t a_bytes = GpuSkinning::kPaletteBytes)
		{
			D3D11_BUFFER_DESC desc{};
			desc.ByteWidth = a_bytes;
			desc.Usage = D3D11_USAGE_DEFAULT;
			desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
			return CreateBufferWithViews(desc, a_name, a_buffer);
		}

		/** @brief A world pose as the batched shader reads it: rotation as w, x, y, z, then the translation. */
		struct StaticWorld
		{
			float rotation[4];
			float translation[4];
		};
		static_assert(sizeof(StaticWorld) == 32);

		struct ActiveRecord
		{
			std::uint32_t slot;
			std::uint32_t hasLast;
		};

		void CopyRows(const void* a_source, std::uint32_t a_boneCount, std::vector<SkinRows>& a_rows)
		{
			a_rows.resize(a_boneCount);
			if (a_source)
				std::memcpy(a_rows.data(), a_source, static_cast<std::size_t>(a_boneCount) * sizeof(SkinRows));
		}
	}

	struct GpuSkinning::Resources
	{
		winrt::com_ptr<ID3D11Buffer> info;
		winrt::com_ptr<ID3D11ShaderResourceView> infoSrv;
		winrt::com_ptr<ID3D11Buffer> current;
		winrt::com_ptr<ID3D11UnorderedAccessView> currentUav;
		winrt::com_ptr<ID3D11ShaderResourceView> currentSrv;
		winrt::com_ptr<ID3D11Buffer> previous;
		winrt::com_ptr<ID3D11UnorderedAccessView> previousUav;
		winrt::com_ptr<ID3D11ShaderResourceView> previousSrv;
		winrt::com_ptr<ID3D11Buffer> last;
		winrt::com_ptr<ID3D11ShaderResourceView> lastSrv;
		winrt::com_ptr<ID3D11Buffer> currentConstants;
		winrt::com_ptr<ID3D11Buffer> previousConstants;
		winrt::com_ptr<ID3D11Buffer> staging;
	};

	SkinRows ExpectedSkinRows(const RE::NiTransform& a_world, const RE::NiTransform& a_skinToBone)
	{
		const auto& w = a_world.rotate.entry;
		const auto& s = a_skinToBone.rotate.entry;
		const float scale = a_world.scale * a_skinToBone.scale;
		float r[3][3];
		for (int row = 0; row < 3; ++row) {
			for (int column = 0; column < 3; ++column)
				r[row][column] = (w[row][0] * s[0][column] + w[row][1] * s[1][column] + w[row][2] * s[2][column]) * scale;
		}
		const RE::NiPoint3 t = a_skinToBone.translate * a_world.scale;
		const RE::NiPoint3 moved{ w[0][0] * t.x + w[0][1] * t.y + w[0][2] * t.z + a_world.translate.x,
			w[1][0] * t.x + w[1][1] * t.y + w[1][2] * t.z + a_world.translate.y,
			w[2][0] * t.x + w[2][1] * t.y + w[2][2] * t.z + a_world.translate.z };
		return { r[0][0], r[0][1], r[0][2], moved.x, r[1][0], r[1][1], r[1][2], moved.y, r[2][0], r[2][1], r[2][2], moved.z };
	}

	struct GpuSkinning::Batch
	{
		std::unique_ptr<Util::LazyShader<ID3D11ComputeShader>> shader;
		std::unique_ptr<ConstantBuffer> constants;
		winrt::com_ptr<ID3D11Buffer> info;
		winrt::com_ptr<ID3D11ShaderResourceView> infoSrv;
		winrt::com_ptr<ID3D11Buffer> active;
		winrt::com_ptr<ID3D11ShaderResourceView> activeSrv;
		winrt::com_ptr<ID3D11Buffer> staticWorlds;
		winrt::com_ptr<ID3D11ShaderResourceView> staticSrv;
		winrt::com_ptr<ID3D11Buffer> current;
		winrt::com_ptr<ID3D11UnorderedAccessView> currentUav;
		winrt::com_ptr<ID3D11ShaderResourceView> currentSrv;
		winrt::com_ptr<ID3D11Buffer> previous;
		winrt::com_ptr<ID3D11UnorderedAccessView> previousUav;
		winrt::com_ptr<ID3D11ShaderResourceView> previousSrv;
		winrt::com_ptr<ID3D11Buffer> currentConstants;
		winrt::com_ptr<ID3D11Buffer> previousConstants;
		winrt::com_ptr<ID3D11DeviceContext1> context1;
		std::vector<bool> slotUsed = std::vector<bool>(kBatchPaletteSlots, false);
		std::vector<std::uint32_t> staticFree;
		std::uint32_t staticNext{};
		std::vector<StaticWorld> staticCpu = std::vector<StaticWorld>(kBatchStaticCapacity);
		std::vector<ActiveRecord> activeCpu;
	};

	GpuSkinning::GpuSkinning() = default;
	GpuSkinning::~GpuSkinning() = default;

	bool GpuSkinning::Register(RE::NiSkinInstance* a_skin, std::uint64_t a_token, std::vector<std::uint32_t> a_slots, std::vector<SkinRows> a_skinToBone)
	{
		if (!a_skin || a_slots.empty() || a_slots.size() != a_skinToBone.size() || a_slots.size() > kMaximumBones)
			return false;
		const std::scoped_lock lock(mutex);
		Entry& entry = entries[a_skin];
		ReleaseBatchSlot(entry);
		entry = {};
		entry.skin.reset(a_skin);
		entry.token = a_token;
		entry.slots = std::move(a_slots);
		entry.skinToBone = std::move(a_skinToBone);
		entry.batchable = a_skin->bones != nullptr && a_skin->skinData != nullptr;
		for (std::size_t index = 0; entry.batchable && index < entry.slots.size(); ++index)
			entry.batchable = a_skin->bones[index] != nullptr;
		entry.batchStatic.assign(entry.slots.size(), kBatchUnusedBone);
		return true;
	}

	void GpuSkinning::RemoveClaim(std::uint64_t a_token)
	{
		const std::scoped_lock lock(mutex);
		for (auto entry = entries.begin(); entry != entries.end();) {
			if (entry->second.token == a_token) {
				ReleaseBatchSlot(entry->second);
				entry = entries.erase(entry);
			} else
				++entry;
		}
		claimStates.erase(a_token);
	}

	void GpuSkinning::SetClaimState(std::uint64_t a_token, bool a_active, winrt::com_ptr<ID3D11ShaderResourceView> a_worlds, std::uint64_t a_solverFrame)
	{
		const std::scoped_lock lock(mutex);
		ClaimState& state = claimStates[a_token];
		state.active = a_active;
		state.worlds = std::move(a_worlds);
		state.solverFrame = a_solverFrame;
	}

	bool GpuSkinning::ArmKinematicCheck(const RE::NiSkinInstance* a_skin)
	{
		const std::scoped_lock lock(mutex);
		const auto found = entries.find(a_skin);
		if (found == entries.end())
			return false;
		found->second.kinematicCheck = {};
		found->second.kinematicCheck.armed = true;
		found->second.stepPoses.clear();
		return true;
	}

	GpuSkinning::KinematicCheck GpuSkinning::GetKinematicCheck(const RE::NiSkinInstance* a_skin) const
	{
		const std::scoped_lock lock(mutex);
		const auto found = entries.find(a_skin);
		return found == entries.end() ? KinematicCheck{} : found->second.kinematicCheck;
	}

	void GpuSkinning::RecordStepPoses(std::uint64_t a_token)
	{
		const std::scoped_lock lock(mutex);
		for (auto& [skin, entry] : entries) {
			if (entry.token != a_token || !entry.kinematicCheck.armed || !entry.skin->bones)
				continue;
			entry.stepPoses.resize(entry.slots.size());
			for (std::size_t index = 0; index < entry.slots.size(); ++index) {
				const RE::NiAVObject* node = entry.skin->bones[index];
				if (entry.slots[index] == kStaticBone && node)
					entry.stepPoses[index] = node->world;
			}
		}
	}

	void GpuSkinning::MeasureKinematicBones(Entry& a_entry, const RE::NiSkinInstance* a_skin)
	{
		if (a_entry.stepPoses.size() != a_entry.slots.size() || !a_skin->boneMatrices || !a_skin->skinData || !a_skin->bones)
			return;
		std::vector<SkinRows> engine;
		CopyRows(a_skin->boneMatrices, static_cast<std::uint32_t>(a_entry.slots.size()), engine);
		KinematicCheck& check = a_entry.kinematicCheck;
		check.staticBones = 0;
		for (std::size_t index = 0; index < a_entry.slots.size(); ++index) {
			const RE::NiAVObject* node = a_skin->bones[index];
			if (a_entry.slots[index] != kStaticBone || !node)
				continue;
			++check.staticBones;
			const RE::NiTransform skinToBone = a_skin->skinData->GetBoneDataSkinToBone(static_cast<std::uint32_t>(index));
			const SkinRows bindRows = ExpectedSkinRows(node->world, skinToBone);
			const SkinRows stepRows = ExpectedSkinRows(a_entry.stepPoses[index], skinToBone);
			for (std::size_t row = 0; row < engine[index].size(); ++row) {
				check.engineVsBindPoseMax = std::max(check.engineVsBindPoseMax, std::abs(engine[index][row] - bindRows[row]));
				check.engineVsStepPoseMax = std::max(check.engineVsStepPoseMax, std::abs(engine[index][row] - stepRows[row]));
			}
			check.stepVsBindTranslationMax = std::max(check.stepVsBindTranslationMax, node->world.translate.GetDistance(a_entry.stepPoses[index].translate));
		}
		++check.samples;
	}

	void GpuSkinning::SetBatched(bool a_batched)
	{
		const std::scoped_lock lock(mutex);
		if (a_batched && !batched)
			batchFailed = false;
		batched = a_batched;
	}

	std::size_t GpuSkinning::BatchedSkinCount() const
	{
		const std::scoped_lock lock(mutex);
		return batchedCount;
	}

	bool GpuSkinning::EnsureBatch()
	{
		if (batchFailed || !globals::d3d::device || !globals::d3d::context)
			return false;
		const auto fail = [this](const char* a_reason) {
			batchFailed = true;
			batchedCount = 0;
			logger::warn("{} the batched skin palette is off, so each skinned mesh builds its own at draw time: {}", kLogPrefix, a_reason);
			return false;
		};
		if (!batch) {
			auto created = std::make_unique<Batch>();
			created->shader = std::make_unique<Util::LazyShader<ID3D11ComputeShader>>();
			try {
				created->constants = std::make_unique<ConstantBuffer>(ConstantBufferDesc<SkinPaletteBatchConstants>(true), "SmpPhysics::SkinPaletteBatchConstants");
			} catch (const std::exception& error) {
				return fail(error.what());
			}
			if (FAILED(globals::d3d::context->QueryInterface(IID_PPV_ARGS(created->context1.put()))) || !created->context1)
				return fail("ID3D11DeviceContext1 is unavailable");

			constexpr std::uint32_t kBoneRecords = kBatchPaletteSlots * kMaximumBones;
			constexpr std::uint32_t kPaletteTotalBytes = kBatchPaletteSlots * kPaletteBytes;
			const D3D11_BUFFER_DESC infoDesc = StructuredBufferDesc<SkinPaletteBatchBone>(kBoneRecords, false, false);
			const D3D11_BUFFER_DESC activeDesc = StructuredBufferDesc<ActiveRecord>(kBatchPaletteSlots, false, true);
			const D3D11_BUFFER_DESC staticDesc = StructuredBufferDesc<StaticWorld>(kBatchStaticCapacity, false, true);
			if (!CreateStructuredBuffer(infoDesc, kBoneRecords, "SmpPhysics::SkinPaletteBatchBones", created->info, created->infoSrv) ||
				!CreateStructuredBuffer(activeDesc, kBatchPaletteSlots, "SmpPhysics::SkinPaletteBatchActive", created->active, created->activeSrv) ||
				!CreateStructuredBuffer(staticDesc, kBatchStaticCapacity, "SmpPhysics::SkinPaletteBatchNodePoses", created->staticWorlds, created->staticSrv))
				return fail("a buffer could not be created");
			if (!CreateRawBuffer(true, "SmpPhysics::SkinPaletteBatchCurrent", created->current, created->currentSrv, &created->currentUav, kPaletteTotalBytes) ||
				!CreateRawBuffer(true, "SmpPhysics::SkinPaletteBatchPrevious", created->previous, created->previousSrv, &created->previousUav, kPaletteTotalBytes) ||
				!CreatePaletteConstants("SmpPhysics::SkinPaletteBatchCurrentConstants", created->currentConstants, kPaletteTotalBytes) ||
				!CreatePaletteConstants("SmpPhysics::SkinPaletteBatchPreviousConstants", created->previousConstants, kPaletteTotalBytes))
				return fail("a palette buffer could not be created");
			batch = std::move(created);
		}
		try {
			const std::vector<std::pair<const char*, const char*>> defines;
			if (!batch->shader->Get(kBatchShaderPath, defines, kShaderTarget, "main", "SmpPhysics::SkinPaletteBatchCS"))
				throw std::runtime_error("compilation failed");
		} catch (const std::exception& error) {
			return fail(error.what());
		}
		return true;
	}

	bool GpuSkinning::AssignBatchSlot(Entry& a_entry)
	{
		std::uint32_t needed = 0;
		for (const std::uint32_t slot : a_entry.slots)
			needed += slot == kStaticBone ? 1u : 0u;
		if (batch->staticNext + needed > kBatchStaticCapacity + static_cast<std::uint32_t>(batch->staticFree.size()))
			return false;
		std::uint32_t chosen = kBatchPaletteSlots;
		for (std::uint32_t index = 0; index < kBatchPaletteSlots; ++index) {
			if (!batch->slotUsed[index]) {
				chosen = index;
				break;
			}
		}
		if (chosen == kBatchPaletteSlots)
			return false;
		batch->slotUsed[chosen] = true;
		a_entry.batchSlot = static_cast<int>(chosen);
		a_entry.batchScales.clear();
		a_entry.batchHasLast = false;
		for (std::size_t bone = 0; bone < a_entry.slots.size(); ++bone) {
			if (a_entry.slots[bone] != kStaticBone) {
				a_entry.batchStatic[bone] = kBatchUnusedBone;
				continue;
			}
			if (!batch->staticFree.empty()) {
				a_entry.batchStatic[bone] = batch->staticFree.back();
				batch->staticFree.pop_back();
			} else {
				a_entry.batchStatic[bone] = batch->staticNext++;
			}
		}
		return true;
	}

	void GpuSkinning::ReleaseBatchSlot(Entry& a_entry)
	{
		if (!batch || a_entry.batchSlot < 0)
			return;
		batch->slotUsed[static_cast<std::size_t>(a_entry.batchSlot)] = false;
		for (std::uint32_t& index : a_entry.batchStatic) {
			if (index != kBatchUnusedBone)
				batch->staticFree.push_back(index);
			index = kBatchUnusedBone;
		}
		a_entry.batchSlot = -1;
		a_entry.batchHasLast = false;
	}

	void GpuSkinning::BuildBatch()
	{
		const std::scoped_lock lock(mutex);
		batchedCount = 0;
		// Every entry's palette is stale until this build re-stamps it, so a frame that builds nothing binds nothing.
		++batchSerial;
		if (!enabled || !batched || batchFailed || entries.empty() || !globals::d3d::context)
			return;

		ID3D11ShaderResourceView* worlds = nullptr;
		std::vector<Entry*> candidates;
		for (auto& [skin, entry] : entries) {
			const auto state = claimStates.find(entry.token);
			const bool eligible = entry.batchable && state != claimStates.end() && state->second.active && state->second.worlds && entry.skin->bones &&
			                      (!worlds || worlds == state->second.worlds.get());
			if (!eligible) {
				entry.batchHasLast = false;
				continue;
			}
			worlds = state->second.worlds.get();
			candidates.push_back(&entry);
		}
		if (candidates.empty() || !EnsureBatch())
			return;

		auto* context = globals::d3d::context;
		batch->activeCpu.clear();
		std::uint32_t highestSlot = 0;
		std::vector<SkinPaletteBatchBone> records(kMaximumBones);
		std::vector<Entry*> built;
		for (Entry* entry : candidates) {
			if (entry->batchSlot < 0 && !AssignBatchSlot(*entry))
				continue;
			const std::size_t boneCount = entry->slots.size();
			std::vector<float> scales(boneCount);
			for (std::size_t bone = 0; bone < boneCount; ++bone) {
				const RE::NiAVObject* node = entry->skin->bones[bone];
				scales[bone] = node ? node->world.scale : 1.0f;
			}
			if (scales != entry->batchScales) {
				for (std::uint32_t bone = 0; bone < kMaximumBones; ++bone) {
					SkinPaletteBatchBone& record = records[bone];
					record = {};
					if (bone >= boneCount) {
						record.slot = kBatchUnusedBone;
						continue;
					}
					const bool node = entry->slots[bone] == kStaticBone;
					record.slot = node ? kBatchNodeBone : entry->slots[bone];
					record.worldScale = scales[bone];
					record.staticIndex = node ? entry->batchStatic[bone] : 0u;
					record.skinToBone = entry->skinToBone[bone];
				}
				const UINT bytes = kMaximumBones * sizeof(SkinPaletteBatchBone);
				const UINT offset = static_cast<UINT>(entry->batchSlot) * bytes;
				const D3D11_BOX box{ offset, 0, 0, offset + bytes, 1, 1 };
				context->UpdateSubresource(batch->info.get(), 0, &box, records.data(), 0, 0);
				entry->batchScales = std::move(scales);
			}
			for (std::size_t bone = 0; bone < boneCount; ++bone) {
				const RE::NiAVObject* boneNode = entry->skin->bones[bone];
				if (entry->slots[bone] != kStaticBone || !boneNode)
					continue;
				const RE::NiTransform& node = boneNode->world;
				const Quat rotation = FromRotationMatrix(node.rotate.entry);
				batch->staticCpu[entry->batchStatic[bone]] = { { rotation.w, rotation.x, rotation.y, rotation.z }, { node.translate.x, node.translate.y, node.translate.z, 0.0f } };
			}
			batch->activeCpu.push_back({ static_cast<std::uint32_t>(entry->batchSlot), entry->batchHasLast ? 1u : 0u });
			highestSlot = std::max(highestSlot, static_cast<std::uint32_t>(entry->batchSlot));
			built.push_back(entry);
		}
		if (batch->activeCpu.empty())
			return;

		if (!UploadDiscard(context, batch->active.get(), batch->activeCpu.data(), batch->activeCpu.size() * sizeof(ActiveRecord)))
			return;
		if (batch->staticNext > 0 && !UploadDiscard(context, batch->staticWorlds.get(), batch->staticCpu.data(), static_cast<std::size_t>(batch->staticNext) * sizeof(StaticWorld)))
			return;

		SkinPaletteBatchConstants values;
		values.activeCount = static_cast<std::uint32_t>(batch->activeCpu.size());
		try {
			batch->constants->Update(values);
		} catch (const std::exception& error) {
			batchFailed = true;
			batchedCount = 0;
			logger::warn("{} the batched skin palette is off, so each skinned mesh builds its own at draw time: {}", kLogPrefix, error.what());
			return;
		}

		{
			CS_GPU_PASS("SmpPhysics::SkinPaletteBatch");
			ID3D11ShaderResourceView* views[4] = { worlds, batch->infoSrv.get(), batch->activeSrv.get(), batch->staticSrv.get() };
			ID3D11UnorderedAccessView* targets[2] = { batch->currentUav.get(), batch->previousUav.get() };
			ID3D11Buffer* constantView = batch->constants->CB();
			context->CSSetShader(batch->shader->get(), nullptr, 0);
			context->CSSetConstantBuffers(0, 1, &constantView);
			context->CSSetShaderResources(0, 4, views);
			context->CSSetUnorderedAccessViews(0, 2, targets, nullptr);
			context->Dispatch(SmpPhysics::DispatchGroups(values.activeCount * kMaximumBones, kThreadsPerGroup), 1, 1);

			ID3D11ShaderResourceView* noViews[4] = {};
			ID3D11UnorderedAccessView* noTargets[2] = {};
			ID3D11Buffer* noConstant = nullptr;
			context->CSSetShaderResources(0, 4, noViews);
			context->CSSetUnorderedAccessViews(0, 2, noTargets, nullptr);
			context->CSSetConstantBuffers(0, 1, &noConstant);
			context->CSSetShader(nullptr, nullptr, 0);
		}

		const D3D11_BOX used{ 0, 0, 0, (highestSlot + 1) * kPaletteBytes, 1, 1 };
		context->CopySubresourceRegion(batch->currentConstants.get(), 0, 0, 0, 0, batch->current.get(), 0, &used);
		context->CopySubresourceRegion(batch->previousConstants.get(), 0, 0, 0, 0, batch->previous.get(), 0, &used);

		for (Entry* entry : built) {
			entry->batchHasLast = true;
			entry->batchSerial = batchSerial;
		}
		batchedCount = built.size();
	}

	std::size_t GpuSkinning::SkinCount() const
	{
		const std::scoped_lock lock(mutex);
		return entries.size();
	}

	std::uint64_t GpuSkinning::TakeAppliedCount()
	{
		const std::scoped_lock lock(mutex);
		return std::exchange(appliedCount, 0ull);
	}

	void GpuSkinning::SetEnabled(bool a_enabled)
	{
		const std::scoped_lock lock(mutex);
		enabled = a_enabled;
	}

	void GpuSkinning::InvalidateShader()
	{
		const std::scoped_lock lock(mutex);
		if (shader)
			shader->Reset();
		if (batch && batch->shader)
			batch->shader->Reset();
		shaderFailed = false;
		batchFailed = false;
	}

	std::vector<Util::ShaderJob> GpuSkinning::ShaderJobs()
	{
		return { { kShaderPath, kShaderTarget, "main" }, { kBatchShaderPath, kShaderTarget, "main" } };
	}

	bool GpuSkinning::EnsureShader()
	{
		if (shaderFailed)
			return false;
		if (!globals::d3d::device)
			return false;
		if (!shader)
			shader = std::make_unique<Util::LazyShader<ID3D11ComputeShader>>();
		const std::vector<std::pair<const char*, const char*>> defines;
		try {
			if (!constants)
				constants = std::make_unique<ConstantBuffer>(ConstantBufferDesc<SkinPaletteConstants>(true), "SmpPhysics::SkinPaletteConstants");
			if (!shader->Get(kShaderPath, defines, kShaderTarget, "main", "SmpPhysics::SkinPaletteCS"))
				throw std::runtime_error("compilation failed");
		} catch (const std::exception& error) {
			shaderFailed = true;
			logger::warn("{} the skinning compute shader did not compile, so hair is drawn from the engine's matrices: {}", kLogPrefix, error.what());
			return false;
		}
		return true;
	}

	bool GpuSkinning::CreateResources(Entry& a_entry)
	{
		if (!globals::d3d::device)
			return false;
		auto resources = std::make_shared<Resources>();

		const D3D11_BUFFER_DESC infoDesc = StructuredBufferDesc<SkinPaletteBone>(kMaximumBones, false, true);
		if (!CreateStructuredBuffer(infoDesc, kMaximumBones, "SmpPhysics::SkinPaletteBones", resources->info, resources->infoSrv) ||
			!CreateRawBuffer(true, "SmpPhysics::SkinPaletteCurrent", resources->current, resources->currentSrv, &resources->currentUav) ||
			!CreateRawBuffer(true, "SmpPhysics::SkinPalettePrevious", resources->previous, resources->previousSrv, &resources->previousUav) ||
			!CreateRawBuffer(false, "SmpPhysics::SkinPaletteLast", resources->last, resources->lastSrv, nullptr) ||
			!CreatePaletteConstants("SmpPhysics::SkinPaletteBoneConstants", resources->currentConstants) ||
			!CreatePaletteConstants("SmpPhysics::SkinPalettePreviousBoneConstants", resources->previousConstants))
			return false;

		a_entry.resources = std::move(resources);
		return true;
	}

	void GpuSkinning::Refresh(Entry& a_entry, const ClaimState& a_state, RE::NiSkinInstance* a_skin)
	{
		auto* context = globals::d3d::context;
		Resources& resources = *a_entry.resources;
		const auto boneCount = static_cast<std::uint32_t>(a_entry.slots.size());

		D3D11_MAPPED_SUBRESOURCE mapped{};
		if (FAILED(context->Map(resources.info.get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped)))
			return;
		auto* records = static_cast<SkinPaletteBone*>(mapped.pData);
		for (std::uint32_t index = 0; index < boneCount; ++index) {
			const RE::NiAVObject* node = a_skin->bones[index];
			SkinPaletteBone& record = records[index];
			record.slot = a_entry.slots[index];
			record.worldScale = node ? node->world.scale : 1.0f;
			record.padding0 = 0;
			record.padding1 = 0;
			record.skinToBone = a_entry.skinToBone[index];
		}
		context->Unmap(resources.info.get(), 0);

		const D3D11_BOX engineBytes{ 0, 0, 0, boneCount * kBytesPerBone, 1, 1 };
		context->UpdateSubresource(resources.current.get(), 0, &engineBytes, a_skin->boneMatrices, 0, 0);
		context->UpdateSubresource(resources.previous.get(), 0, &engineBytes, a_skin->prevBoneMatrices, 0, 0);

		SkinPaletteConstants values;
		values.boneCount = boneCount;
		values.hasLast = a_entry.hasLast ? 1u : 0u;
		constants->Update(values);

		{
			CS_GPU_PASS("SmpPhysics::SkinPalette");
			ID3D11ShaderResourceView* views[3] = { a_state.worlds.get(), resources.infoSrv.get(), resources.lastSrv.get() };
			ID3D11UnorderedAccessView* targets[2] = { resources.currentUav.get(), resources.previousUav.get() };
			ID3D11Buffer* constantView = constants->CB();
			context->CSSetShader(shader->get(), nullptr, 0);
			context->CSSetConstantBuffers(0, 1, &constantView);
			context->CSSetShaderResources(0, 3, views);
			context->CSSetUnorderedAccessViews(0, 2, targets, nullptr);
			context->Dispatch(SmpPhysics::DispatchGroups(boneCount, kThreadsPerGroup), 1, 1);

			ID3D11ShaderResourceView* noViews[3] = {};
			ID3D11UnorderedAccessView* noTargets[2] = {};
			ID3D11Buffer* noConstant = nullptr;
			context->CSSetShaderResources(0, 3, noViews);
			context->CSSetUnorderedAccessViews(0, 2, noTargets, nullptr);
			context->CSSetConstantBuffers(0, 1, &noConstant);
			context->CSSetShader(nullptr, nullptr, 0);
		}

		context->CopyResource(resources.currentConstants.get(), resources.current.get());
		context->CopyResource(resources.previousConstants.get(), resources.previous.get());
		context->CopyResource(resources.last.get(), resources.current.get());
		a_entry.hasLast = true;
		a_entry.lastEngineFrame = a_skin->frameID;
		a_entry.lastSolverFrame = a_state.solverFrame;

		if (a_entry.captureRequested)
			ReadCapture(a_entry, a_state, a_skin);
	}

	void GpuSkinning::ReadCapture(Entry& a_entry, const ClaimState& a_state, RE::NiSkinInstance* a_skin)
	{
		auto* context = globals::d3d::context;
		Resources& resources = *a_entry.resources;
		a_entry.captureRequested = false;
		if (!resources.staging) {
			const D3D11_BUFFER_DESC desc = StagingBufferDesc(kPaletteBytes);
			if (FAILED(globals::d3d::device->CreateBuffer(&desc, nullptr, resources.staging.put())))
				return;
			Util::SetResourceName(resources.staging.get(), "SmpPhysics::SkinPaletteStaging");
		}

		const auto boneCount = static_cast<std::uint32_t>(a_entry.slots.size());
		Capture capture;
		capture.boneCount = boneCount;
		capture.slots = a_entry.slots;
		const auto read = [&](ID3D11Buffer* a_source, std::vector<SkinRows>& a_rows) {
			context->CopyResource(resources.staging.get(), a_source);
			D3D11_MAPPED_SUBRESOURCE mapped{};
			if (FAILED(context->Map(resources.staging.get(), 0, D3D11_MAP_READ, 0, &mapped)))
				return false;
			CopyRows(mapped.pData, boneCount, a_rows);
			context->Unmap(resources.staging.get(), 0);
			return true;
		};
		if (!read(resources.currentConstants.get(), capture.current) || !read(resources.previousConstants.get(), capture.previous))
			return;
		CopyRows(a_skin->boneMatrices, boneCount, capture.engineCurrent);
		CopyRows(a_skin->prevBoneMatrices, boneCount, capture.enginePrevious);
		if (a_entry.batchSlot >= 0 && batch && a_entry.batchSerial == batchSerial) {
			const UINT offset = static_cast<UINT>(a_entry.batchSlot) * kPaletteBytes;
			const D3D11_BOX box{ offset, 0, 0, offset + boneCount * kBytesPerBone, 1, 1 };
			context->CopySubresourceRegion(resources.staging.get(), 0, 0, 0, 0, batch->currentConstants.get(), 0, &box);
			D3D11_MAPPED_SUBRESOURCE batchMapped{};
			if (SUCCEEDED(context->Map(resources.staging.get(), 0, D3D11_MAP_READ, 0, &batchMapped))) {
				CopyRows(batchMapped.pData, boneCount, capture.batchCurrent);
				context->Unmap(resources.staging.get(), 0);
			}
		}

		capture.reference = capture.engineCurrent;
		winrt::com_ptr<ID3D11Resource> worldsResource;
		a_state.worlds->GetResource(worldsResource.put());
		const auto worldsBuffer = worldsResource.try_as<ID3D11Buffer>();
		if (worldsBuffer) {
			D3D11_BUFFER_DESC worldsDesc{};
			worldsBuffer->GetDesc(&worldsDesc);
			const D3D11_BUFFER_DESC stagingDesc = StagingBufferDesc(worldsDesc.ByteWidth);
			winrt::com_ptr<ID3D11Buffer> worldsStaging;
			D3D11_MAPPED_SUBRESOURCE mapped{};
			if (SUCCEEDED(globals::d3d::device->CreateBuffer(&stagingDesc, nullptr, worldsStaging.put()))) {
				Util::SetResourceName(worldsStaging.get(), "SmpPhysics::SkinCaptureWorldsStaging");
				context->CopyResource(worldsStaging.get(), worldsBuffer.get());
				if (SUCCEEDED(context->Map(worldsStaging.get(), 0, D3D11_MAP_READ, 0, &mapped))) {
					const auto* worlds = static_cast<const float*>(mapped.pData);
					for (std::uint32_t index = 0; index < boneCount; ++index) {
						const std::uint32_t slot = a_entry.slots[index];
						if (slot == kStaticBone || (slot + 1u) * kFloatsPerWorldRecord * sizeof(float) > worldsDesc.ByteWidth)
							continue;
						const float* record = worlds + static_cast<std::size_t>(slot) * kFloatsPerWorldRecord;
						const Transform world{ Quat{ record[0], record[1], record[2], record[3] }, { record[4], record[5], record[6] } };
						const RE::NiAVObject* node = a_skin->bones[index];
						capture.reference[index] = ComposeSkinRows(world, node ? node->world.scale : 1.0f, a_entry.skinToBone[index]);
					}
					context->Unmap(worldsStaging.get(), 0);
				}
			}
		}
		capture.valid = true;
		a_entry.capture = std::move(capture);
	}

	void GpuSkinning::Apply(RE::NiSkinInstance* a_skin)
	{
		const std::scoped_lock lock(mutex);
		if (!enabled || entries.empty() || !a_skin)
			return;
		const auto found = entries.find(a_skin);
		if (found == entries.end())
			return;
		Entry& entry = found->second;
		if (entry.kinematicCheck.armed)
			MeasureKinematicBones(entry, a_skin);
		if (batched && entry.batchable && entry.batchSlot >= 0 && entry.batchSerial == batchSerial && batch && batch->context1 && !entry.captureRequested) {
			constexpr UINT kPaletteConstants = kPaletteBytes / kConstantRegisterBytes;
			const UINT first = static_cast<UINT>(entry.batchSlot) * kPaletteConstants;
			ID3D11Buffer* current = batch->currentConstants.get();
			ID3D11Buffer* previous = batch->previousConstants.get();
			batch->context1->VSSetConstantBuffers1(kEngineCurrentSlot, 1, &current, &first, &kPaletteConstants);
			batch->context1->VSSetConstantBuffers1(kEnginePreviousSlot, 1, &previous, &first, &kPaletteConstants);
			if (globals::game::isVR) {
				// VR's binding routine skips VSSetConstantBuffers when its cached buffer is unchanged, so the cache has to follow ours.
				auto& cached = globals::game::shadowState->GetVRRuntimeData().VSConstantBuffers;
				cached[kEngineCurrentSlot] = Util::AsW32(current);
				cached[kEnginePreviousSlot] = Util::AsW32(previous);
			}
			++appliedCount;
			return;
		}
		const auto stateFound = claimStates.find(entry.token);
		if (stateFound == claimStates.end() || !stateFound->second.active || !stateFound->second.worlds)
			return;
		if (!a_skin->boneMatrices || !a_skin->prevBoneMatrices || !a_skin->bones || entry.resourcesFailed || !globals::d3d::context)
			return;
		if (!EnsureShader())
			return;
		if (!entry.resources && !CreateResources(entry)) {
			entry.resourcesFailed = true;
			logger::warn("{} could not create the skinning buffers for a mesh of claim {}, so it is drawn from the engine's matrices", kLogPrefix, entry.token);
			return;
		}

		const ClaimState& state = stateFound->second;
		if (entry.lastEngineFrame != a_skin->frameID || entry.lastSolverFrame != state.solverFrame || entry.captureRequested) {
			try {
				Refresh(entry, state, a_skin);
			} catch (const std::exception& error) {
				entry.resourcesFailed = true;
				logger::warn("{} the skin palette of a mesh of claim {} could not be built, so it is drawn from the engine's matrices: {}", kLogPrefix, entry.token, error.what());
				return;
			}
		}
		if (!entry.hasLast)
			return;

		auto* context = globals::d3d::context;
		ID3D11Buffer* current = entry.resources->currentConstants.get();
		ID3D11Buffer* previous = entry.resources->previousConstants.get();
		context->VSSetConstantBuffers(kEngineCurrentSlot, 1, &current);
		context->VSSetConstantBuffers(kEnginePreviousSlot, 1, &previous);
		if (globals::game::isVR) {
			// VR's binding routine skips VSSetConstantBuffers when its cached buffer is unchanged, so the cache has to follow ours.
			auto& cached = globals::game::shadowState->GetVRRuntimeData().VSConstantBuffers;
			cached[kEngineCurrentSlot] = Util::AsW32(current);
			cached[kEnginePreviousSlot] = Util::AsW32(previous);
		}
		++appliedCount;
	}

	GpuSkinning::Capture GpuSkinning::RequestCapture(const RE::NiSkinInstance* a_skin)
	{
		const std::scoped_lock lock(mutex);
		const auto found = entries.find(a_skin);
		if (found == entries.end())
			return {};
		found->second.captureRequested = true;
		return found->second.capture;
	}
}
