/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 MOLA georeferencing module. Tools for geo-referencing datasets.

 Copyright (C) 2024-2026 Jose Luis Blanco, University of Almeria
 SPDX-License-Identifier: GPL-3.0-or-later
*/

/**
 * @file   mola-mm-add-geodetic-cli.cpp
 * @brief  CLI tool to add geodetic coordinates (lat/lon/alt) to metric map point clouds
 * @author Jose Luis Blanco Claraco
 * @date   Jan 20, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/topography/conversions.h>
#include <mrpt/topography/data_types.h>
#include <mrpt/version.h>

#include <iostream>

namespace
{
// CLI flags:
struct Cli
{
    TCLAP::CmdLine cmd{"mola-mm-add-geodetic"};

    TCLAP::ValueArg<std::string> argInputMap{
        "i", "input", "Input metric map file (*.mm)", true, "input.mm", "input.mm", cmd};

    TCLAP::ValueArg<std::string> argOutputMap{
        "o",  "output",    "Output metric map file (*.mm) with geodetic coordinates added",
        true, "output.mm", "output.mm",
        cmd};

    TCLAP::ValueArg<std::string> argGeoRefFile{
        "g",
        "georef",
        "Optional geo-referencing file (*.georef or *.yaml) to use if the input map "
        "does not have embedded georeferencing information",
        false,
        "",
        "map.georef",
        cmd};

    TCLAP::MultiArg<std::string> argLayers{
        "l",
        "layer",
        "Layer(s) to process. If not provided, all CGenericPointsMap layers will be processed. "
        "This argument can appear multiple times.",
        false,
        "layerName",
        cmd};

    TCLAP::ValueArg<std::string> argPlugins{
        "p",
        "load-plugins",
        "One or more (comma separated) *.so files to load as plugins",
        false,
        "",
        "foobar.so",
        cmd};

    TCLAP::SwitchArg argVerbose{
        "v", "verbose", "Enable verbose output with progress information", cmd, false};
};

bool is_binary_file(const std::string& fil)
{
    return mrpt::system::extractFileExtension(fil) == "georef";
}

std::optional<mp2p_icp::metric_map_t::Georeferencing> load_georef_file(const std::string& filePath)
{
    std::cout << "[mola-mm-add-geodetic] Loading geo-referencing data from: '" << filePath << "'..."
              << "\n";

    std::optional<mp2p_icp::metric_map_t::Georeferencing> g;

    if (is_binary_file(filePath))
    {
        // Binary .georef file
        mrpt::io::CFileGZInputStream f(filePath);
        auto                         arch = mrpt::serialization::archiveFrom(f);
        arch >> g;
    }
    else
    {
        // YAML file
        const auto yamlData = mrpt::containers::yaml::FromFile(filePath);
        g                   = mp2p_icp::FromYAML(yamlData);
    }

    return g;
}

void add_geodetic_fields_to_layer(
    mrpt::maps::CPointsMap& pointMap, const mp2p_icp::metric_map_t::Georeferencing& georef,
    bool verbose)
{
    const size_t numPoints = pointMap.size();

    if (verbose)
    {
        std::cout << "  Processing " << numPoints << " points..."
                  << "\n";
    }

    // Create the double fields for geodetic coordinates:
    pointMap.registerField_double("latitude");
    pointMap.registerField_double("longitude");
    pointMap.registerField_double("altitude");

    auto* latitudes  = pointMap.getPointsBufferRef_double_field("latitude");
    auto* longitudes = pointMap.getPointsBufferRef_double_field("longitude");
    auto* altitudes  = pointMap.getPointsBufferRef_double_field("altitude");

    ASSERT_(latitudes != nullptr);
    ASSERT_(longitudes != nullptr);
    ASSERT_(altitudes != nullptr);

    // Allow re-runing on existing fields:
    latitudes->assign(numPoints, 0.0);
    longitudes->assign(numPoints, 0.0);
    altitudes->assign(numPoints, 0.0);

    // Get point coordinates in map frame
    const auto& xs = pointMap.getPointsBufferRef_x();
    const auto& ys = pointMap.getPointsBufferRef_y();
    const auto& zs = pointMap.getPointsBufferRef_z();

    // Transform map->ENU using the inverse of T_enu_to_map
    const auto T_map_to_enu = -georef.T_enu_to_map.mean;

    // Process each point
    size_t progressCounter = 0;
    for (size_t i = 0; i < numPoints; i++)
    {
        // Point in map frame
        const mrpt::math::TPoint3D ptMap(xs[i], ys[i], zs[i]);

        // Transform to ENU frame
        mrpt::math::TPoint3D ptENU;
        T_map_to_enu.composePoint(ptMap, ptENU);

        try
        {
            // Convert ENU to geocentric coordinates
            mrpt::topography::TGeocentricCoords geocentricPt;
            mrpt::topography::ENUToGeocentric(
                ptENU, georef.geo_coord, geocentricPt,
                mrpt::topography::TEllipsoid::Ellipsoid_WGS84());

            // Convert geocentric to geodetic (WGS84)
            mrpt::topography::TGeodeticCoords geodeticCoords;
            mrpt::topography::geocentricToGeodetic(
                geocentricPt, geodeticCoords, mrpt::topography::TEllipsoid::Ellipsoid_WGS84());

            // Store in double fields (in decimal degrees and meters)
            (*latitudes)[i]  = geodeticCoords.lat.getDecimalValue();
            (*longitudes)[i] = geodeticCoords.lon.getDecimalValue();
            (*altitudes)[i]  = geodeticCoords.height;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Warning: Failed to convert point " << i
                      << " to geodetic coordinates: " << e.what() << "\n";
            // Set to zero on error
            (*latitudes)[i]  = 0.0;
            (*longitudes)[i] = 0.0;
            (*altitudes)[i]  = 0.0;
        }

        // Progress reporting for large point clouds
        if (verbose && numPoints > 10000)
        {
            progressCounter++;
            if (progressCounter % 10000 == 0)
            {
                std::cout << "    Processed " << progressCounter << " / " << numPoints
                          << " points ("
                          << (100.0 * static_cast<double>(progressCounter) /
                              static_cast<double>(numPoints))
                          << "%)"
                          << "\n";
            }
        }
    }

    if (verbose)
    {
        std::cout << "  Done processing layer."
                  << "\n";
    }
}

void run_add_geodetic(Cli& cli)
{
    const bool verbose = cli.argVerbose.getValue();

    // Load plugins if specified
    if (cli.argPlugins.isSet())
    {
        std::string errMsg;
        const auto  plugins = cli.argPlugins.getValue();
        std::cout << "[mola-mm-add-geodetic] Loading plugin(s): " << plugins << "\n";
        if (!mrpt::system::loadPluginModules(plugins, errMsg))
        {
            throw std::runtime_error("Failed to load plugins: " + errMsg);
        }
    }

    // Load input map
    const auto& filInput = cli.argInputMap.getValue();
    std::cout << "[mola-mm-add-geodetic] Reading input map from: '" << filInput << "'..."
              << "\n";

    mp2p_icp::metric_map_t mm;
    if (!mm.load_from_file(filInput))
    {
        throw std::runtime_error("Failed to load input map file: " + filInput);
    }

    std::cout << "[mola-mm-add-geodetic] Loaded map. Contents:\n" << mm.contents_summary() << "\n";

    // Get georeferencing information
    std::optional<mp2p_icp::metric_map_t::Georeferencing> georef;

    if (mm.georeferencing.has_value())
    {
        std::cout << "[mola-mm-add-geodetic] Using embedded georeferencing from map."
                  << "\n";
        georef = mm.georeferencing;
    }
    else if (cli.argGeoRefFile.isSet())
    {
        georef = load_georef_file(cli.argGeoRefFile.getValue());
        if (!georef.has_value())
        {
            throw std::runtime_error(
                "Failed to load georeferencing from file: " + cli.argGeoRefFile.getValue());
        }
    }
    else
    {
        throw std::runtime_error(
            "Input map does not have embedded georeferencing and no --georef file was provided. "
            "Use the --georef flag to specify a georeferencing file.");
    }

    // Display georeferencing info
    if (verbose)
    {
        std::cout << "[mola-mm-add-geodetic] Georeferencing information:"
                  << "\n";
        std::cout << "  Reference geodetic coordinates (WGS84):"
                  << "\n";
        std::cout << "    Latitude:  " << georef->geo_coord.lat.getAsString() << "\n";
        std::cout << "    Longitude: " << georef->geo_coord.lon.getAsString() << "\n";
        std::cout << "    Height:    " << georef->geo_coord.height << " m"
                  << "\n";
    }

    // We may have a .georef file, estimated from IMU only so the ENU frame is defined, but
    // without geodetic coordinates. Then, returns and exist without creating a new output map:
    if (georef->geo_coord.isClear())
    {
        std::cout << "[mola-mm-add-geodetic] Georeferencing file does not contain valid geodetic "
                     "coordinates. No geodetic fields will be added.\n";
        std::cout << "[mola-mm-add-geodetic] Exiting without creating output map.\n";
        return;
    }

    // Determine which layers to process
    std::vector<std::string> layersToProcess;
    if (cli.argLayers.isSet())
    {
        // Process only selected layers
        for (const auto& layerName : cli.argLayers.getValue())
        {
            layersToProcess.push_back(layerName);
        }
    }
    else
    {
        // Process all layers
        for (const auto& [name, layer] : mm.layers)
        {
            layersToProcess.push_back(name);
        }
    }

    // Process each layer
    size_t processedLayers = 0;
    for (const auto& layerName : layersToProcess)
    {
        auto it = mm.layers.find(layerName);
        if (it == mm.layers.end())
        {
            std::cerr << "Warning: Layer '" << layerName << "' not found in map. Skipping."
                      << "\n";
            continue;
        }

        auto* pointMap = dynamic_cast<mrpt::maps::CPointsMap*>(it->second.get());
        if (!pointMap)
        {
            std::cout << "Skipping layer '" << layerName
                      << "' (not a CPointsMap-derived type, type is: "
                      << it->second->GetRuntimeClass()->className << ")"
                      << "\n";
            continue;
        }

        std::cout << "[mola-mm-add-geodetic] Processing layer: '" << layerName << "'..."
                  << "\n";

        add_geodetic_fields_to_layer(*pointMap, *georef, verbose);
        processedLayers++;
    }

    std::cout << "[mola-mm-add-geodetic] Processed " << processedLayers << " layer(s)."
              << "\n";

    // Save output map
    const auto& filOutput = cli.argOutputMap.getValue();
    std::cout << "[mola-mm-add-geodetic] Saving output map to: '" << filOutput << "'..."
              << "\n";

    if (!mm.save_to_file(filOutput))
    {
        throw std::runtime_error("Failed to save output map file: " + filOutput);
    }

    std::cout << "[mola-mm-add-geodetic] Done! Output map saved successfully."
              << "\n";
}

}  // namespace

int main(int argc, char** argv)
{
    try
    {
        Cli cli;

        // Parse arguments
        if (!cli.cmd.parse(argc, argv))
        {
            return 1;
        }

        run_add_geodetic(cli);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}