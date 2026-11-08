-- GPUSMPPhysicsNG xmake build script
-- SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

set_xmakever("2.8.2")

set_project("GPUSMPPhysicsNG")
set_license("GPL-3.0")

local version = "0.1.0"
set_version(version)

set_languages("c++23")
set_warnings("allextra")
set_policy("package.requires_lock", true)
add_rules("mode.debug", "mode.releasedbg")
set_defaultmode("releasedbg")

add_requires("catch2", "nlohmann_json")

-- Engine-free solver core: rig building, packing, collision, skinning math.
-- No CommonLib, no D3D; this is what the Catch2 suite exercises.
target("GPUSMPPhysicsNG-Core")
set_kind("static")
add_includedirs("src", { public = true })
add_files("src/smp/*.cpp")
add_headerfiles("src/**.h")
target_end()

-- xmake build GPUSMPPhysicsNG-Tests && xmake run GPUSMPPhysicsNG-Tests
target("GPUSMPPhysicsNG-Tests")
set_kind("binary")
set_default(false)
set_group("tests")
add_deps("GPUSMPPhysicsNG-Core")
add_packages("catch2", "nlohmann_json")
add_files("tests/*.cpp")
add_includedirs("tests")
add_defines('SMPPHYSICS_FIXTURE_DIR="' .. path.join(os.projectdir(), "tests", "fixtures"):gsub("\\", "/") .. '"')
add_defines('SMPPHYSICS_SHADER_DIR="' .. path.join(os.projectdir(), "shaders"):gsub("\\", "/") .. '"')
add_defines('SMPPHYSICS_FEATURE_SHADER_DIR="' .. path.join(os.projectdir(), "shaders"):gsub("\\", "/") .. '"')
add_defines("WIN32_LEAN_AND_MEAN", "NOMINMAX")
add_syslinks("d3dcompiler", "d3d11", "dxgi")
