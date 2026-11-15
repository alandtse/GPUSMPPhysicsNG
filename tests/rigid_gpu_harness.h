#pragma once

// Shared Direct3D plumbing for the rigid-body compute-kernel tests: the device, the kernel compile, one packed step's
// buffers and views, and a GPU timer. The WARP replay test and the hardware benchmark dispatch the same kernels.

#include "Utils/D3D.h"
#include "shader_cache_dir.h"
#include "smp/RigidGpuPacking.h"

#include <catch2/catch_test_macros.hpp>

#include <d3d11.h>
#include <d3dcompiler.h>
#include <wrl/client.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

namespace SmpPhysics::RigidGpuTest
{
	using Microsoft::WRL::ComPtr;
	using SmpPhysics::RigidGpu::StepData;

	/** @brief Writes the shader's integrated bodies back into a_world: world transform and both velocities. */
	inline void UnpackBodies(const std::vector<RigidGpu::BodyRecord>& a_bodies, RigidReference::World& a_world)
	{
		const std::size_t count = std::min(a_bodies.size(), a_world.bodies.size());
		for (std::size_t index = 0; index < count; ++index) {
			const RigidGpu::BodyRecord& record = a_bodies[index];
			RigidReference::Body& body = a_world.bodies[index];
			body.worldTransform.basis = RigidReference::Mat3{
				RigidReference::Vec3{ record.basis0.x, record.basis0.y, record.basis0.z },
				RigidReference::Vec3{ record.basis1.x, record.basis1.y, record.basis1.z },
				RigidReference::Vec3{ record.basis2.x, record.basis2.y, record.basis2.z },
			};
			body.worldTransform.origin = { record.origin.x, record.origin.y, record.origin.z };
			body.linearVelocity = { record.linearVelocity.x, record.linearVelocity.y, record.linearVelocity.z };
			body.angularVelocity = { record.angularVelocity.x, record.angularVelocity.y, record.angularVelocity.z };
		}
	}

	/** @brief Threads one group of the bulk kernels runs: every entry point but the coloured solve is compiled that wide. */
	inline constexpr UINT kThreadsPerGroup{ 64 };

	inline std::string Hex(HRESULT a_result)
	{
		constexpr char kDigits[] = "0123456789ABCDEF";
		std::string text = "0x";
		const auto value = static_cast<std::uint32_t>(a_result);
		for (int shift = 28; shift >= 0; shift -= 4)
			text.push_back(kDigits[(value >> shift) & 0xFu]);
		return text;
	}

	struct GpuDevice
	{
		ComPtr<ID3D11Device> device;
		ComPtr<ID3D11DeviceContext> context;
	};

	/** @brief Creates a feature-level-11_0 device on the given driver, or fails the test with the HRESULT. */
	inline GpuDevice CreateDevice(D3D_DRIVER_TYPE a_driver)
	{
		GpuDevice gpu;
		const D3D_FEATURE_LEVEL levels[] = { D3D_FEATURE_LEVEL_11_0 };
		const HRESULT result = D3D11CreateDevice(nullptr, a_driver, nullptr, 0, levels, 1, D3D11_SDK_VERSION,
			gpu.device.GetAddressOf(), nullptr, gpu.context.GetAddressOf());
		INFO("D3D11CreateDevice driver " << static_cast<int>(a_driver) << " returned " << Hex(result));
		REQUIRE(result == S_OK);
		return gpu;
	}

	inline std::filesystem::path KernelPath()
	{
		return std::filesystem::path(SMPPHYSICS_FEATURE_SHADER_DIR) / "SmpPhysics" / "RigidSolverCS.hlsl";
	}

	/** @brief Compiles one entry point through the plugin's own path, so its blob cache keeps the test run short. */
	inline ComPtr<ID3D11ComputeShader> CompileKernel(ID3D11Device* a_device, const char* a_entry,
		const std::filesystem::path& a_path = KernelPath())
	{
		Util::SetShaderRoot(std::filesystem::path(SMPPHYSICS_SHADER_DIR).wstring());
		Util::SetShaderCacheDir(TestShaderCacheDir());
		const winrt::com_ptr<ID3DBlob> code = Util::CompileShaderBlob(a_path.c_str(), {}, "cs_5_0", a_entry);
		REQUIRE(code);

		ComPtr<ID3D11ComputeShader> shader;
		INFO("CreateComputeShader returned " << Hex(a_device->CreateComputeShader(code->GetBufferPointer(), code->GetBufferSize(), nullptr, shader.GetAddressOf())));
		REQUIRE(shader);
		return shader;
	}

	/** @brief Every kernel one step dispatches, both solve variants included, compiled for one device. */
	struct Kernels
	{
		ComPtr<ID3D11ComputeShader> predict;
		ComPtr<ID3D11ComputeShader> setup;
		ComPtr<ID3D11ComputeShader> solve;
		ComPtr<ID3D11ComputeShader> integrate;
		ComPtr<ID3D11ComputeShader> solveColored;
	};

	inline Kernels CompileKernels(ID3D11Device* a_device)
	{
		Kernels kernels;
		kernels.predict = CompileKernel(a_device, "RigidPredict");
		kernels.setup = CompileKernel(a_device, "RigidSetup");
		kernels.solve = CompileKernel(a_device, "RigidSolveIsland");
		kernels.integrate = CompileKernel(a_device, "RigidIntegrate");
		kernels.solveColored = CompileKernel(a_device, "RigidSolveIslandColored");
		return kernels;
	}

	inline UINT GroupCount(std::size_t a_threads)
	{
		return static_cast<UINT>(std::max<std::size_t>((a_threads + kThreadsPerGroup - 1) / kThreadsPerGroup, 1));
	}

	inline ComPtr<ID3D11Buffer> CreateStructured(ID3D11Device* a_device, UINT a_stride, std::size_t a_count, UINT a_bind,
		const void* a_data = nullptr)
	{
		D3D11_BUFFER_DESC desc{};
		desc.ByteWidth = a_stride * static_cast<UINT>(std::max<std::size_t>(a_count, 1));
		desc.Usage = D3D11_USAGE_DEFAULT;
		desc.BindFlags = a_bind;
		desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
		desc.StructureByteStride = a_stride;
		D3D11_SUBRESOURCE_DATA initial{};
		initial.pSysMem = a_data;
		ComPtr<ID3D11Buffer> buffer;
		const HRESULT result = a_device->CreateBuffer(&desc, a_data ? &initial : nullptr, buffer.GetAddressOf());
		INFO("CreateBuffer(" << a_stride << " x " << a_count << ") returned " << Hex(result));
		REQUIRE(result == S_OK);
		return buffer;
	}

	template <typename T>
	inline ComPtr<ID3D11Buffer> CreateTyped(ID3D11Device* a_device, const std::vector<T>& a_values, UINT a_bind)
	{
		return CreateStructured(a_device, static_cast<UINT>(sizeof(T)), a_values.size(), a_bind, a_values.empty() ? nullptr : a_values.data());
	}

	inline ComPtr<ID3D11UnorderedAccessView> CreateUav(ID3D11Device* a_device, ID3D11Buffer* a_buffer, std::size_t a_count)
	{
		D3D11_UNORDERED_ACCESS_VIEW_DESC desc{};
		desc.Format = DXGI_FORMAT_UNKNOWN;
		desc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
		desc.Buffer.FirstElement = 0;
		desc.Buffer.NumElements = static_cast<UINT>(std::max<std::size_t>(a_count, 1));
		ComPtr<ID3D11UnorderedAccessView> view;
		REQUIRE(a_device->CreateUnorderedAccessView(a_buffer, &desc, view.GetAddressOf()) == S_OK);
		return view;
	}

	inline ComPtr<ID3D11ShaderResourceView> CreateSrv(ID3D11Device* a_device, ID3D11Buffer* a_buffer, std::size_t a_count)
	{
		D3D11_SHADER_RESOURCE_VIEW_DESC desc{};
		desc.Format = DXGI_FORMAT_UNKNOWN;
		desc.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
		desc.Buffer.FirstElement = 0;
		desc.Buffer.NumElements = static_cast<UINT>(std::max<std::size_t>(a_count, 1));
		ComPtr<ID3D11ShaderResourceView> view;
		REQUIRE(a_device->CreateShaderResourceView(a_buffer, &desc, view.GetAddressOf()) == S_OK);
		return view;
	}

	/** @brief Reads a buffer back to the host, which also waits for the kernels to finish. */
	template <typename T>
	inline std::vector<T> ReadBack(ID3D11Device* a_device, ID3D11DeviceContext* a_context, ID3D11Buffer* a_buffer, std::size_t a_count)
	{
		D3D11_BUFFER_DESC desc{};
		a_buffer->GetDesc(&desc);
		desc.Usage = D3D11_USAGE_STAGING;
		desc.BindFlags = 0;
		desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
		ComPtr<ID3D11Buffer> staging;
		REQUIRE(a_device->CreateBuffer(&desc, nullptr, staging.GetAddressOf()) == S_OK);
		a_context->CopyResource(staging.Get(), a_buffer);

		std::vector<T> values(a_count);
		D3D11_MAPPED_SUBRESOURCE mapped{};
		REQUIRE(a_context->Map(staging.Get(), 0, D3D11_MAP_READ, 0, &mapped) == S_OK);
		std::memcpy(values.data(), mapped.pData, sizeof(T) * a_count);
		a_context->Unmap(staging.Get(), 0);
		return values;
	}

	/** @brief One packed step's buffers and views, which the caller dispatches the rigid kernels over. */
	class StepBuffers
	{
	public:
		StepBuffers(const GpuDevice& a_gpu, const StepData& a_step) :
			device(a_gpu.device.Get()), context(a_gpu.context.Get()), bodyCount(a_step.bodies.size())
		{
			ID3D11Device* gpu = device;
			bodies = CreateTyped(gpu, a_step.bodies, D3D11_BIND_UNORDERED_ACCESS);
			solverBodies = CreateTyped(gpu, a_step.solverBodies, D3D11_BIND_UNORDERED_ACCESS);
			rows = CreateTyped(gpu, a_step.rows, D3D11_BIND_UNORDERED_ACCESS);
			rowMasks = CreateTyped(gpu, a_step.rowMasks, D3D11_BIND_UNORDERED_ACCESS);
			constraints = CreateTyped(gpu, a_step.constraints, D3D11_BIND_SHADER_RESOURCE);
			islands = CreateTyped(gpu, a_step.islands, D3D11_BIND_SHADER_RESOURCE);
			islandConstraints = CreateTyped(gpu, a_step.islandConstraints, D3D11_BIND_SHADER_RESOURCE);
			coloredConstraints = CreateTyped(gpu, a_step.coloredConstraints, D3D11_BIND_SHADER_RESOURCE);
			colorOffsets = CreateTyped(gpu, a_step.colorOffsets, D3D11_BIND_SHADER_RESOURCE);
			contacts = CreateTyped(gpu, a_step.contacts, D3D11_BIND_SHADER_RESOURCE);

			D3D11_BUFFER_DESC constantsDesc{};
			constantsDesc.ByteWidth = sizeof(SmpPhysics::RigidGpu::StepConstants);
			constantsDesc.Usage = D3D11_USAGE_IMMUTABLE;
			constantsDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
			D3D11_SUBRESOURCE_DATA constantsData{};
			constantsData.pSysMem = &a_step.constants;
			REQUIRE(gpu->CreateBuffer(&constantsDesc, &constantsData, constantBuffer.GetAddressOf()) == S_OK);

			uavs.push_back(CreateUav(gpu, bodies.Get(), a_step.bodies.size()));
			uavs.push_back(CreateUav(gpu, solverBodies.Get(), a_step.solverBodies.size()));
			uavs.push_back(CreateUav(gpu, rows.Get(), a_step.rows.size()));
			uavs.push_back(CreateUav(gpu, rowMasks.Get(), a_step.rowMasks.size()));

			resources.push_back(CreateSrv(gpu, constraints.Get(), a_step.constraints.size()));
			resources.push_back(CreateSrv(gpu, islands.Get(), a_step.islands.size()));
			resources.push_back(CreateSrv(gpu, islandConstraints.Get(), a_step.islandConstraints.size()));
			resources.push_back(CreateSrv(gpu, coloredConstraints.Get(), a_step.coloredConstraints.size()));
			resources.push_back(CreateSrv(gpu, colorOffsets.Get(), a_step.colorOffsets.size()));
			resources.emplace_back();  // t5 is the GPU solver's contact buffer, which a replay has none of
			resources.push_back(CreateSrv(gpu, contacts.Get(), a_step.contacts.size()));
		}

		StepBuffers(const StepBuffers&) = delete;
		StepBuffers& operator=(const StepBuffers&) = delete;

		/** @brief Binds the constant buffer and every view the four rigid kernels read or write. */
		void Bind() const
		{
			std::vector<ID3D11UnorderedAccessView*> uavPointers;
			for (const ComPtr<ID3D11UnorderedAccessView>& view : uavs)
				uavPointers.push_back(view.Get());
			std::vector<ID3D11ShaderResourceView*> resourcePointers;
			for (const ComPtr<ID3D11ShaderResourceView>& view : resources)
				resourcePointers.push_back(view.Get());

			context->CSSetConstantBuffers(0, 1, constantBuffer.GetAddressOf());
			context->CSSetUnorderedAccessViews(0, static_cast<UINT>(uavPointers.size()), uavPointers.data(), nullptr);
			context->CSSetShaderResources(0, static_cast<UINT>(resourcePointers.size()), resourcePointers.data());
		}

		/** @brief Unbinds everything, so the body buffer can be read back. */
		void Unbind() const
		{
			std::vector<ID3D11UnorderedAccessView*> uavPointers(uavs.size());
			std::vector<ID3D11ShaderResourceView*> resourcePointers(resources.size());
			context->CSSetUnorderedAccessViews(0, static_cast<UINT>(uavPointers.size()), uavPointers.data(), nullptr);
			context->CSSetShaderResources(0, static_cast<UINT>(resourcePointers.size()), resourcePointers.data());
			context->CSSetShader(nullptr, nullptr, 0);
		}

		/** @brief Reads the integrated bodies back to the host, which waits for the dispatches to finish. */
		[[nodiscard]] std::vector<SmpPhysics::RigidGpu::BodyRecord> ReadBodies() const
		{
			return ReadBack<SmpPhysics::RigidGpu::BodyRecord>(device, context, bodies.Get(), bodyCount);
		}

	private:
		ID3D11Device* device;
		ID3D11DeviceContext* context;
		std::size_t bodyCount{};
		ComPtr<ID3D11Buffer> bodies;
		ComPtr<ID3D11Buffer> solverBodies;
		ComPtr<ID3D11Buffer> rows;
		ComPtr<ID3D11Buffer> rowMasks;
		ComPtr<ID3D11Buffer> constraints;
		ComPtr<ID3D11Buffer> islands;
		ComPtr<ID3D11Buffer> islandConstraints;
		ComPtr<ID3D11Buffer> coloredConstraints;
		ComPtr<ID3D11Buffer> colorOffsets;
		ComPtr<ID3D11Buffer> contacts;
		ComPtr<ID3D11Buffer> constantBuffer;
		std::vector<ComPtr<ID3D11UnorderedAccessView>> uavs;
		std::vector<ComPtr<ID3D11ShaderResourceView>> resources;
	};

	/** @brief Dispatches one step's four kernels in the reference's order, a_solve covering one group per island. */
	inline void DispatchStep(ID3D11DeviceContext* a_context, const Kernels& a_kernels, const StepData& a_step,
		ID3D11ComputeShader* a_solve, UINT a_solveGroups)
	{
		a_context->CSSetShader(a_kernels.predict.Get(), nullptr, 0);
		a_context->Dispatch(GroupCount(a_step.bodies.size()), 1, 1);
		a_context->CSSetShader(a_kernels.setup.Get(), nullptr, 0);
		a_context->Dispatch(GroupCount(a_step.constraints.size()), 1, 1);
		a_context->CSSetShader(a_solve, nullptr, 0);
		a_context->Dispatch(a_solveGroups, 1, 1);
		a_context->CSSetShader(a_kernels.integrate.Get(), nullptr, 0);
		a_context->Dispatch(GroupCount(a_step.bodies.size()), 1, 1);
	}

	/** @brief Runs the four kernels over one packed step and returns the bodies they left behind. */
	inline std::vector<SmpPhysics::RigidGpu::BodyRecord> RunStep(const GpuDevice& a_gpu, const Kernels& a_kernels,
		const StepData& a_step, ID3D11ComputeShader* a_solve, UINT a_solveGroups)
	{
		StepBuffers buffers(a_gpu, a_step);
		buffers.Bind();
		DispatchStep(a_gpu.context.Get(), a_kernels, a_step, a_solve, a_solveGroups);
		buffers.Unbind();
		return buffers.ReadBodies();
	}

	/** @brief One measured region's GPU time, from a timestamp pair wrapped in a disjoint query. */
	class GpuTimer
	{
	public:
		explicit GpuTimer(const GpuDevice& a_gpu) :
			context(a_gpu.context.Get())
		{
			ID3D11Device* device = a_gpu.device.Get();
			D3D11_QUERY_DESC timestamp{};
			timestamp.Query = D3D11_QUERY_TIMESTAMP;
			D3D11_QUERY_DESC disjoint{};
			disjoint.Query = D3D11_QUERY_TIMESTAMP_DISJOINT;
			REQUIRE(device->CreateQuery(&timestamp, start.GetAddressOf()) == S_OK);
			REQUIRE(device->CreateQuery(&timestamp, finish.GetAddressOf()) == S_OK);
			REQUIRE(device->CreateQuery(&disjoint, this->disjoint.GetAddressOf()) == S_OK);
		}

		GpuTimer(const GpuTimer&) = delete;
		GpuTimer& operator=(const GpuTimer&) = delete;

		void Begin() const
		{
			context->Begin(disjoint.Get());
			context->End(start.Get());
		}

		/** @brief Waits for the region and returns its milliseconds, failing the test if the GPU clock was disrupted. */
		[[nodiscard]] double End() const
		{
			context->End(finish.Get());
			context->End(disjoint.Get());
			while (context->GetData(disjoint.Get(), nullptr, 0, 0) == S_FALSE) {
			}
			D3D11_QUERY_DATA_TIMESTAMP_DISJOINT data{};
			REQUIRE(context->GetData(disjoint.Get(), &data, sizeof(data), 0) == S_OK);
			REQUIRE(!data.Disjoint);
			UINT64 begin = 0;
			UINT64 end = 0;
			REQUIRE(context->GetData(start.Get(), &begin, sizeof(begin), 0) == S_OK);
			REQUIRE(context->GetData(finish.Get(), &end, sizeof(end), 0) == S_OK);
			return static_cast<double>(end - begin) * 1000.0 / static_cast<double>(data.Frequency);
		}

	private:
		ID3D11DeviceContext* context;
		ComPtr<ID3D11Query> start;
		ComPtr<ID3D11Query> finish;
		ComPtr<ID3D11Query> disjoint;
	};
}
