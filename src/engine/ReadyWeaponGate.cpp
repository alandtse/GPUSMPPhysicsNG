#include "engine/ReadyWeaponGate.h"

#include "engine/ReadyWeaponGateLogic.h"

#include "Globals.h"

#include <atomic>
#include <optional>

namespace SmpPhysics::ReadyWeaponGate
{
	namespace
	{
		std::atomic<std::uint32_t> heldHands{ 0 };
		std::atomic<std::uint32_t> swallowed{ 0 };
		std::atomic<bool> installed{ false };
		PressTracker pressTracker;
		GripLatch gripLatch;

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
			if (a_event.IsDown() || a_event.IsUp())
				gripLatch.Set(*hand, a_event.IsDown());
			if (a_event.IsDown()) {
				const bool swallow = pressTracker.Press(*hand, heldHands.load(std::memory_order_relaxed));
				logger::info("[GPU SMP Physics] {} grip pressed, {}", *hand == 0 ? "left" : "right", swallow ? "kept from the weapon" : "passed to the game");
				return swallow;
			}
			return pressTracker.Follow(*hand, a_event.IsUp());
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
		auto* ui = RE::UI::GetSingleton();
		return gripLatch.IsClosed(a_isLeft ? 0 : 1, ui && ui->GameIsPaused());
	}

	std::uint32_t SwallowedCount() noexcept
	{
		return swallowed.load(std::memory_order_relaxed);
	}
}
