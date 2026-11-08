#include "engine/ReadyWeaponGate.h"

#include "Globals.h"

#include <openvr.h>

#include <array>
#include <atomic>
#include <optional>

namespace SmpPhysics::ReadyWeaponGate
{
	namespace
	{
		std::atomic<std::uint32_t> heldHands{ 0 };
		std::atomic<std::uint32_t> swallowed{ 0 };
		std::atomic<bool> installed{ false };
		constexpr std::size_t kGripAxis = 2;          ///< The controller axis that carries grip squeeze on Oculus-style controllers.
		constexpr float kGripAxisClosedAbove = 0.6f;  ///< Squeeze past which the grip counts as closed when the button bit is not set.
		/** Per hand: whether the grip press the game last saw from it was swallowed, so its release is too. */
		std::array<std::atomic<bool>, 2> pressSwallowed{};

		/** The right controller is the one the game calls primary; left-handed mode swaps the roles inside the game, not the devices. */
		std::optional<std::size_t> HandOf(RE::INPUT_DEVICE a_device)
		{
			switch (a_device) {
			case RE::INPUT_DEVICE::kVivePrimary:
			case RE::INPUT_DEVICE::kOculusPrimary:
			case RE::INPUT_DEVICE::kWMRPrimary:
				return 1;
			case RE::INPUT_DEVICE::kViveSecondary:
			case RE::INPUT_DEVICE::kOculusSecondary:
			case RE::INPUT_DEVICE::kWMRSecondary:
				return 0;
			default:
				return std::nullopt;
			}
		}

		bool ShouldSwallow(const RE::ButtonEvent& a_event)
		{
			const auto hand = HandOf(a_event.GetDevice());
			if (!hand)
				return false;
			if (a_event.IsDown()) {
				const std::uint32_t bit = *hand == 0 ? kLeftHand : kRightHand;
				const bool swallow = (heldHands.load(std::memory_order_relaxed) & bit) != 0;
				pressSwallowed[*hand].store(swallow, std::memory_order_relaxed);
				logger::info("[GPU SMP Physics] {} grip pressed, {}", *hand == 0 ? "left" : "right", swallow ? "kept from the weapon" : "passed to the game");
				return swallow;
			}
			const bool swallow = pressSwallowed[*hand].load(std::memory_order_relaxed);
			if (a_event.IsUp())
				pressSwallowed[*hand].store(false, std::memory_order_relaxed);
			return swallow;
		}

		struct ProcessButtonHook
		{
			static void thunk(RE::ReadyWeaponHandler* a_this, RE::ButtonEvent* a_event, RE::PlayerControlsData* a_data)
			{
				if (a_event && ShouldSwallow(*a_event)) {
					swallowed.fetch_add(1, std::memory_order_relaxed);
					return;
				}
				func(a_this, a_event, a_data);
			}
			static inline REL::Relocation<decltype(thunk)> func;
		};
	}

	void Install()
	{
		if (!globals::game::isVR || installed.exchange(true))
			return;
		stl::write_vfunc<0x4, ProcessButtonHook>(RE::VTABLE_ReadyWeaponHandler[0]);
		logger::info("[GPU SMP Physics] Ready Weapon gate installed, so a grip on hair does not draw or sheathe the weapon");
	}

	void SetHeldHands(std::uint32_t a_mask) noexcept
	{
		heldHands.store(a_mask, std::memory_order_relaxed);
	}

	bool IsGripDown(bool a_isLeft) noexcept
	{
		auto* system = globals::game::isVR ? RE::BSOpenVR::GetIVRSystem() : nullptr;
		if (!system)
			return false;
		const vr::TrackedDeviceIndex_t device = system->GetTrackedDeviceIndexForControllerRole(a_isLeft ? vr::TrackedControllerRole_LeftHand : vr::TrackedControllerRole_RightHand);
		vr::VRControllerState_t state{};
		if (device == vr::k_unTrackedDeviceIndexInvalid || !system->GetControllerState(device, &state, sizeof(state)))
			return false;
		return (state.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_Grip)) != 0 || state.rAxis[kGripAxis].x > kGripAxisClosedAbove;
	}

	std::uint32_t SwallowedCount() noexcept
	{
		return swallowed.load(std::memory_order_relaxed);
	}
}
