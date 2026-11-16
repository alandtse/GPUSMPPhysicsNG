-- GPUSMPPhysicsNG xmake build script
-- SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

set_xmakever("2.8.2")

set_config("rex_ini", true)

includes("lib/commonlibsse-ng")

set_project("GPUSMPPhysicsNG")
set_license("GPL-3.0")

local version = "1.0.1"
set_version(version)

set_languages("c++23")
set_warnings("allextra")
set_policy("package.requires_lock", true)
add_rules("mode.debug", "mode.releasedbg")
set_defaultmode("releasedbg")

-- Local package repo: pins the SKSE Menu Framework client header to an exact upstream version.
add_repositories("gsp-pkgs xmake-pkgs")

add_requires("skse-menu-framework-api 3.7.0")
add_requires("catch2", "nlohmann_json", "thread-pool", "directxtk", "microsoft-detours")

-- Opt-in profiling: xmake f --tracy=y builds the real Tracy client (v0.15.0 to match the profiler server; on demand, localhost only); off, zones compile away.
option("tracy")
set_default(false)
set_showmenu(true)
set_description("Build with the Tracy profiler client")
option_end()
if has_config("tracy") then
    add_requires(
        "tracy v0.15.0",
        { configs = { cmake = false, on_demand = true, only_localhost = true, fibers = false } }
    )
end

-- Engine-free solver core: rig building, packing, collision, skinning math.
-- No CommonLib, no D3D; this is what the Catch2 suite exercises.
target("GPUSMPPhysicsNG-Core")
set_kind("static")
add_includedirs("src", { public = true })
add_files("src/smp/*.cpp")
add_headerfiles("src/**.h")
target_end()

-- D3D11 layer: buffers, shader compile, rigid solver dispatch.
target("GPUSMPPhysicsNG-D3D")
set_kind("static")
add_deps("GPUSMPPhysicsNG-Core")
add_packages("thread-pool")
add_includedirs("src", { public = true })
add_files("src/Utils/D3D.cpp", "src/d3d/*.cpp")
if has_config("tracy") then
    add_packages("tracy", { public = true })
    add_defines("TRACY_ENABLE", { public = true })
end
add_defines("WIN32_LEAN_AND_MEAN", "NOMINMAX", { public = true })
add_syslinks("d3dcompiler", "d3d11", "dxgi", "windowsapp")
target_end()

-- Settings page, PCH-free so the SKSE Menu Framework client header never meets the plugin's own imgui types.
target("GPUSMPPhysicsNG-UI")
set_kind("static")
set_warnings("all")
add_deps("commonlibsse-ng")
add_packages("skse-menu-framework-api")
add_defines("_SILENCE_CXX17_CODECVT_HEADER_DEPRECATION_WARNING", "UNICODE", "_UNICODE", "SMP_USE_SKSE_LOG")
add_files("src/ui/UI.cpp")
add_includedirs("src")
target_end()

-- The SKSE plugin: engine-coupled layer plus plugin glue. Built against CommonLibVR (NG branch).
target("GPUSMPPhysicsNG")
add_deps("commonlibsse-ng", "GPUSMPPhysicsNG-D3D", "GPUSMPPhysicsNG-UI")
add_packages("thread-pool", "nlohmann_json", "directxtk", "microsoft-detours")
set_basename("GPUSMPPhysicsNG")
add_defines("SMP_USE_SKSE_LOG")
add_shflags("/DEBUG", "/PDBALTPATH:%_PDB%", { force = true })
add_rules("commonlibsse-ng.plugin", {
    name = "GPUSMPPhysicsNG",
    author = "alandtse",
    description = "GPU SMP physics for hair and cloth, a companion to hdtSMP64",
})
add_files(
    "src/engine/*.cpp",
    "src/plugin/*.cpp",
    "src/Features/*.cpp",
    "src/Features/SmpPhysics/*.cpp",
    "src/Features/Wind/*.cpp",
    "src/Utils/TestBench/*.cpp",
    "vendor/include/devbench/DevBenchAPI.cpp",
    "src/Utils/*.cpp|D3D.cpp"
)
add_headerfiles("src/**.h")
add_includedirs("src", "vendor/include", "vendor/include/devbench")
if has_config("tracy") then
    add_packages("tracy")
else
    add_includedirs("vendor/tracy-stub")
end
add_syslinks("d3dcompiler", "shell32", "version")
set_pcxxheader("src/pch.h")

-- Opt-in deploy (GPUSMP_DEPLOY=1): SkyrimVRPluginTargets / SkyrimPluginTargets
-- (semicolon-separated Data folders or mod-manager mod folders), else
-- SkyrimVRPath (install root or Data folder). No-op when none is set.
after_build(function(target)
    local function to_data_folder(p)
        p = p:trim()
        if os.isdir(path.join(p, "Data")) then
            return path.join(p, "Data")
        end
        return p
    end

    local function nonblank(v)
        if v and v:trim() ~= "" then
            return v
        end
        return nil
    end

    local function collect_targets()
        local explicit = nonblank(os.getenv("SkyrimVRPluginTargets")) or nonblank(os.getenv("SkyrimPluginTargets"))
        if explicit then
            local out = {}
            for _, dir in ipairs(explicit:split(";")) do
                dir = dir:trim()
                if dir ~= "" then
                    table.insert(out, dir)
                end
            end
            return out
        end
        local vr_root = os.getenv("SkyrimVRPath")
        if vr_root and vr_root ~= "" then
            return { to_data_folder(vr_root) }
        end
        return {}
    end

    if os.getenv("GPUSMP_DEPLOY") ~= "1" then
        print("Deploy skipped (set GPUSMP_DEPLOY=1 to copy into the Skyrim targets)")
        return
    end
    local targets = collect_targets()
    if #targets == 0 then
        print("No deploy target (set SkyrimVRPath, SkyrimVRPluginTargets, or SkyrimPluginTargets)")
        return
    end

    local dll = target:targetfile()
    local pdb = target:symbolfile()
    for _, dir in ipairs(targets) do
        local dest = path.join(dir, "SKSE", "Plugins")
        os.mkdir(dest)
        local shaderDest = path.join(dest, "GPUSMPPhysicsNG", "Shaders")
        os.mkdir(shaderDest)
        os.cp(path.join(os.projectdir(), "shaders", "*"), shaderDest)
        -- A running game holds the DLL/PDB locks; skip with a warning.
        try({
            function()
                os.cp(dll, dest)
                if os.isfile(pdb) then
                    os.cp(pdb, dest)
                end
                print("Deployed DLL to " .. dest)
            end,
            catch({
                function()
                    print("Skipped DLL for " .. dest .. " (game running?)")
                end,
            }),
        })
    end
end)
target_end()

-- xmake build GPUSMPPhysicsNG-Tests && xmake run GPUSMPPhysicsNG-Tests
target("GPUSMPPhysicsNG-Tests")
set_kind("binary")
set_default(false)
set_group("tests")
add_deps("GPUSMPPhysicsNG-Core", "GPUSMPPhysicsNG-D3D")
add_packages("catch2", "nlohmann_json", "thread-pool")
add_files("tests/*.cpp")
add_files(
    "src/Utils/TestBench/CrowdBases.cpp",
    "src/Utils/TestBench/CrowdFilter.cpp",
    "src/Utils/TestBench/TestCrowdLayout.cpp",
    "src/Features/SmpPhysics/CrowdStyles.cpp",
    "src/Features/Wind/WindShelter.cpp"
)
add_includedirs("tests")
add_defines('SMPPHYSICS_FIXTURE_DIR="' .. path.join(os.projectdir(), "tests", "fixtures"):gsub("\\", "/") .. '"')
add_defines('SMPPHYSICS_SHADER_DIR="' .. path.join(os.projectdir(), "shaders"):gsub("\\", "/") .. '"')
add_defines('SMPPHYSICS_FEATURE_SHADER_DIR="' .. path.join(os.projectdir(), "shaders"):gsub("\\", "/") .. '"')
add_defines("WIN32_LEAN_AND_MEAN", "NOMINMAX")
add_syslinks("d3dcompiler", "d3d11", "dxgi")
