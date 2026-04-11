// -----------------------------------------------------------------------------
//   A Modular Optimization framework for Localization and mApping  (MOLA)
//
// Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
// Licensed under the GNU GPL v3.
//
// This file is part of MOLA.
// MOLA is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// MOLA. If not, see <https://www.gnu.org/licenses/>.
//
// Closed-source licenses available upon request, for this odometry package
// alone or in combination with the complete SLAM system.
// -----------------------------------------------------------------------------

#include <mola_sm_loop_closure/LoopClosureInterface.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

// CLI flags:

struct Cli
{
    TCLAP::CmdLine cmd{"mola-sm-lc-cli"};

    TCLAP::ValueArg<std::string> argInput{
        "i", "input", "Input .simplemap file", true, "map.simplemap", "map.simplemap", cmd};

    TCLAP::ValueArg<std::string> argOutput{
        "o",
        "output",
        "Output .simplemap file to write to",
        true,
        "corrected_map.simplemap",
        "corrected_map.simplemap",
        cmd};

    TCLAP::ValueArg<std::string> argPlugins{
        "l",
        "load-plugins",
        "One or more (comma separated) *.so files to load as plugins, e.g. "
        "defining new CMetricMap classes",
        false,
        "foobar.so",
        "foobar.so",
        cmd};

    TCLAP::ValueArg<std::string> argPipeline{
        "p",  "pipeline",          "YAML file with the loop closure algorithm configuration file.",
        true, "loop_closure.yaml", "loop_closure.yaml",
        cmd};

    TCLAP::ValueArg<std::string> arg_verbosity_level{
        "v",    "verbosity", "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)", false, "",
        "INFO", cmd};

    TCLAP::ValueArg<std::string> arg_algo{
        "a",
        "algorithm",
        "C++ class name of the loop-closure algorithm to use.",
        false,
        "mola::SimplemapLoopClosure",
        "ClassName",
        cmd};

    TCLAP::ValueArg<std::string> arg_lazy_load_base_dir{
        "",
        "externals-dir",
        "Lazy-load base directory for datasets with externally-stored "
        "observations. If not defined, the program will try anyway to "
        "autodetect any directory side-by-side to the input .simplemap with "
        "the postfix '_Images' and try to use it as lazy-load base directory.",
        false,
        "dataset_Images",
        "<ExternalsDirectory>",
        cmd};
};

namespace
{
void run_sm_to_mm(Cli& cli)
{
    if (cli.argPlugins.isSet())
    {
        std::string sErrs;
        bool        ok = mrpt::system::loadPluginModules(cli.argPlugins.getValue(), sErrs);
        if (!ok)
        {
            std::cerr << "Errors loading plugins: " << cli.argPlugins.getValue() << "\n";
            throw std::runtime_error(sErrs.c_str());
        }
    }

    const auto filYaml = cli.argPipeline.getValue();
    ASSERT_FILE_EXISTS_(filYaml);
    auto yamlData = mola::load_yaml_file(filYaml);

    const auto& filSM = cli.argInput.getValue();

    mrpt::maps::CSimpleMap sm;

    std::cout << "[mola-sm-lc-cli] Reading simplemap from: '" << filSM << "'...\n";

    sm.loadFromFile(filSM);

    std::cout << "[mola-sm-lc-cli] Done read simplemap with " << sm.size() << " keyframes.\n";
    ASSERT_(!sm.empty());

    // Create algorithm:
    auto algoPtr = mrpt::rtti::classFactory(cli.arg_algo.getValue());
    if (!algoPtr)
    {
        THROW_EXCEPTION_FMT(
            "Unregistered algorithm C++ class: '%s'", cli.arg_algo.getValue().c_str());
    }
    auto lcPtr = std::dynamic_pointer_cast<mola::LoopClosureInterface>(algoPtr);
    if (!lcPtr)
    {
        THROW_EXCEPTION_FMT(
            "Algorithm C++ class seems not to be an implementation of 'LoopClosureInterface': '%s'",
            cli.arg_algo.getValue().c_str());
    }
    auto& lc = *lcPtr;

    mrpt::system::VerbosityLevel logLevel = mrpt::system::LVL_INFO;
    if (cli.arg_verbosity_level.isSet())
    {
        using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
        logLevel = vl::name2value(cli.arg_verbosity_level.getValue());
    }

    // Set "params.debug_files_prefix" so generated .tum files, etc. have the expected prefix:
    if (yamlData.has("params"))
    {
        auto debugFilesPrefix = mrpt::system::pathJoin(
            {mrpt::system::extractFileDirectory(filSM),
             mrpt::system::extractFileName(filSM) + "_lc_"});

        yamlData["params"]["debug_files_prefix"] = debugFilesPrefix;
    }

    lc.setMinLoggingLevel(logLevel);

    lc.initialize(yamlData);

    // try to detect lazy load:
    std::string lazyLoadBaseDir;
    if (cli.arg_lazy_load_base_dir.isSet())
    {  // use provided dir:
        lazyLoadBaseDir = cli.arg_lazy_load_base_dir.getValue();
    }
    else
    {  // try to autodetect:
        auto candidateDir = mrpt::system::pathJoin(
            {mrpt::system::extractFileDirectory(filSM),
             mrpt::system::extractFileName(filSM) + "_Images"});
        if (mrpt::system::directoryExists(candidateDir))
        {
            lazyLoadBaseDir = candidateDir;

            std::cout << "[mola-sm-lc-cli] Found lazy-load base directory: '" << candidateDir
                      << "'\n";
        }
    }

    if (!lazyLoadBaseDir.empty())
    {
        mrpt::io::setLazyLoadPathBase(lazyLoadBaseDir);
    }

    // Main stuff here:
    lc.process(sm);

    // save output:
    const auto filOut = cli.argOutput.getValue();
    std::cout << "[mola-sm-lc-cli] Writing output map to: '" << filOut << "'...\n";

    sm.saveToFile(filOut);

    std::cout << "[mola-sm-lc-cli] Done.\n";
}
}  // namespace

int main(int argc, char** argv)
{
    try
    {
        Cli cli;

        // Parse arguments:
        if (!cli.cmd.parse(argc, argv))
        {
            return 1;  // should exit.
        }

        run_sm_to_mm(cli);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }
    return 0;
}
