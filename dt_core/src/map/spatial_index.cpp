#include "map/spatial_index.hpp"
#include <iostream>
#include <filesystem>
#include "gdal_priv.h"
#include "ogrsf_frmts.h"
#include "../../include/map/gdal_initializer.hpp"

/**
 * @brief Internal auxiliary function to inject vertices of a linear ring into the distance R-Tree.
 * @param ring Pointer to the linear ring (OGRLinearRing) containing the geometry vertices.
 * @param rtree Reference to the segment R-Tree where the bounding boxes will be inserted.
 */
void inject_ring_into_rtree(OGRLinearRing* ring, bgi::rtree<RTreeSegmentValue, bgi::quadratic<16>>& rtree) {
    if (!ring) return;
    for (int i = 0; i < ring->getNumPoints() - 1; i++) {
        BgPoint p1(ring->getX(i), ring->getY(i));
        BgPoint p2(ring->getX(i+1), ring->getY(i+1));
        BgSegment segment(p1, p2);
        
        BgBox box;
        bg::envelope(segment, box); 
        rtree.insert(std::make_pair(box, segment));
    }
}

// DATA LOADING (MARGIN AND NAVMESH)

/**
 * @brief Loads, processes, and builds spatial R-Trees from Shapefiles on disk.
 * @param margin_shp Path to the `.shp` file regarding the safety margin.
 * @param mesh_shp Path to the `.shp` file regarding the navigation mesh.
 * @return true if loading and indexing were successful, false otherwise.
 */
bool SpatialIndex::load_shapefiles(const std::string& margin_shp, const std::string& mesh_shp) {
    rtree_margin_.clear();
    rtree_mesh_.clear();
    mesh_triangles_.clear();

    // LOADS SAFETY MARGIN
    GDALDataset* ds_margin = (GDALDataset*)GDALOpenEx(margin_shp.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    if (ds_margin) {
        OGRLayer* layer_margin = ds_margin->GetLayer(0);
        if (layer_margin) {
            OGRFeature* feature;
            layer_margin->ResetReading();
            while ((feature = layer_margin->GetNextFeature()) != nullptr) {
                OGRGeometry* geom = feature->GetGeometryRef();
                if (geom) {
                    OGRwkbGeometryType type = wkbFlatten(geom->getGeometryType());
                    if (type == wkbPolygon) {
                        OGRPolygon* poly = (OGRPolygon*)geom;
                        inject_ring_into_rtree(poly->getExteriorRing(), rtree_margin_);
                    } else if (type == wkbMultiPolygon) {
                        OGRMultiPolygon* mpoly = (OGRMultiPolygon*)geom;
                        for (int i = 0; i < mpoly->getNumGeometries(); i++) {
                            OGRGeometry* sub_geom = mpoly->getGeometryRef(i);
                            if (sub_geom && wkbFlatten(sub_geom->getGeometryType()) == wkbPolygon) {
                                inject_ring_into_rtree(((OGRPolygon*)sub_geom)->getExteriorRing(), rtree_margin_);
                            }
                        }
                    }
                }
                OGRFeature::DestroyFeature(feature);
            }
        }
        GDALClose(ds_margin);
    } else {
        std::cerr << "[SpatialIndex] Error opening Margin: " << margin_shp << "\n";
    }

    // LOADS NAVIGATION MESH
    GDALDataset* ds_mesh = (GDALDataset*)GDALOpenEx(mesh_shp.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    if (ds_mesh) {
        OGRLayer* layer_mesh = ds_mesh->GetLayer(0);
        if (layer_mesh) {
            OGRFeature* feature;
            layer_mesh->ResetReading();
            while ((feature = layer_mesh->GetNextFeature()) != nullptr) {
                OGRGeometry* geom = feature->GetGeometryRef();
                if (geom && wkbFlatten(geom->getGeometryType()) == wkbPolygon) {
                    OGRPolygon* poly = (OGRPolygon*)geom;
                    OGRLinearRing* ext_ring = poly->getExteriorRing();
                    
                    if (ext_ring) {
                        BgPolygon boost_triangle;
                        for (int i = 0; i < ext_ring->getNumPoints(); i++) {
                            bg::append(boost_triangle.outer(), BgPoint(ext_ring->getX(i), ext_ring->getY(i)));
                        }
                        bg::correct(boost_triangle);
                        
                        size_t index = mesh_triangles_.size();
                        mesh_triangles_.push_back(boost_triangle);

                        BgBox triangle_box;
                        bg::envelope(boost_triangle, triangle_box);
                        rtree_mesh_.insert(std::make_pair(triangle_box, index));
                    }
                }
                OGRFeature::DestroyFeature(feature);
            }
        }
        GDALClose(ds_mesh);
    } else {
        std::cerr << "[SpatialIndex] Error opening Mesh: " << mesh_shp << "\n";
    }

    std::cout << "[SpatialIndex] R-Trees Ready! Margin Segments: " << rtree_margin_.size() 
              << " | NavMesh Triangles: " << rtree_mesh_.size() << "\n";
    return true;
}

// FAST MATHEMATICAL CALCULATION FUNCTIONS

/**
 * @brief Auxiliary method that calculates the exact distance in meters to the safety margin.
 * @param position Reference position in `types::Point` format.
 * @return Shortest geometric distance calculated via R-Tree.
 */
double SpatialIndex::calculate_margin_distance(const types::Point& position) const {
    if (rtree_margin_.empty()) return -1.0; 

    BgPoint search_point(position.get_x(), position.get_y());
    std::vector<RTreeSegmentValue> results;
    
    // Queries the 50 closest boxes to ensure the correct one is found
    rtree_margin_.query(bgi::nearest(search_point, 50), std::back_inserter(results));
    
    if (results.empty()) return -1.0;

    // Performs exact point-to-segment measurement for the 50 candidates
    double exact_min_distance = std::numeric_limits<double>::max();
    for (const auto& res : results) {
        double dist = bg::distance(search_point, res.second);
        if (dist < exact_min_distance) {
            exact_min_distance = dist;
        }
    }
    
    return exact_min_distance;
}

/**
 * @brief Evaluates whether a specific point strictly belongs to the navigable polygon.
 * @param position Test coordinates.
 * @return true if the point is contained within the navigable mesh, false otherwise.
 */
bool SpatialIndex::is_navigable(const types::Point& position) const {
    if (rtree_mesh_.empty()) return false;

    BgPoint search_point(position.get_x(), position.get_y());
    std::vector<RTreePolygonValue> candidates;
    
    // Queries the R-Tree for geometrically intercepted triangles
    rtree_mesh_.query(bgi::intersects(search_point), std::back_inserter(candidates));
    
    for (const auto& candidate : candidates) {
        size_t index = candidate.second; 
        if (bg::within(search_point, mesh_triangles_[index])) {
            return true; // Point is strictly inside a safe navigable triangle
        }
    }
    return false; // Outside the navigation mesh
}

// VISUAL DEBUGGING OF R-TREES AND DISTANCE LINES FOR NAVIGABLE POINTS

/**
 * @brief Generates spatial debugging files to validate R-Tree queries via QGIS.
 * @param test_points Vector of sample points for proximity testing.
 * @param output_folder Save directory for the generated vector files.
 * @param epsg_utm EPSG code of the applied UTM cartographic projection.
 */
void SpatialIndex::export_rtree_debug(const std::vector<types::Point>& test_points, const std::string& output_folder, int epsg_utm) const {
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
    OGRSpatialReference srs; 
    srs.importFromEPSG(epsg_utm);

    // SAFETY MARGIN R-TREE
    if (!rtree_margin_.empty()) {
        std::string path_margin = output_folder + "/98_RTree_Caixas_Margem.shp";
        if (std::filesystem::exists(path_margin)) driver->Delete(path_margin.c_str());
        GDALDataset* ds_margin = driver->Create(path_margin.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        OGRLayer* layer_margin = ds_margin->CreateLayer("caixas_margem", &srs, wkbPolygon, nullptr);

        for (auto it = rtree_margin_.begin(); it != rtree_margin_.end(); ++it) {
            BgBox box = it->first;
            double min_x = box.min_corner().get<0>(), min_y = box.min_corner().get<1>();
            double max_x = box.max_corner().get<0>(), max_y = box.max_corner().get<1>();

            OGRPolygon poly; OGRLinearRing ring;
            ring.addPoint(min_x, min_y); ring.addPoint(max_x, min_y);
            ring.addPoint(max_x, max_y); ring.addPoint(min_x, max_y); ring.addPoint(min_x, min_y);
            poly.addRing(&ring);

            OGRFeature* feat = OGRFeature::CreateFeature(layer_margin->GetLayerDefn());
            feat->SetGeometry(&poly);
            if (layer_margin->CreateFeature(feat) != OGRERR_NONE) {
                std::cerr << "[SpatialIndex] Warning: Failed to draw margin debug box.\n";
            }
            OGRFeature::DestroyFeature(feat);
        }
        layer_margin->SyncToDisk();
        GDALClose(ds_margin);
    }

    // NAVIGATION MESH R-TREE
    if (!rtree_mesh_.empty()) {
        std::string path_mesh = output_folder + "/97_RTree_Caixas_Malha.shp";
        if (std::filesystem::exists(path_mesh)) driver->Delete(path_mesh.c_str());
        GDALDataset* ds_mesh = driver->Create(path_mesh.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        OGRLayer* layer_mesh = ds_mesh->CreateLayer("caixas_malha", &srs, wkbPolygon, nullptr);

        for (auto it = rtree_mesh_.begin(); it != rtree_mesh_.end(); ++it) {
            BgBox box = it->first;
            double min_x = box.min_corner().get<0>(), min_y = box.min_corner().get<1>();
            double max_x = box.max_corner().get<0>(), max_y = box.max_corner().get<1>();

            OGRPolygon poly; OGRLinearRing ring;
            ring.addPoint(min_x, min_y); ring.addPoint(max_x, min_y);
            ring.addPoint(max_x, max_y); ring.addPoint(min_x, max_y); ring.addPoint(min_x, min_y);
            poly.addRing(&ring);

            OGRFeature* feat = OGRFeature::CreateFeature(layer_mesh->GetLayerDefn());
            feat->SetGeometry(&poly);
            if (layer_mesh->CreateFeature(feat) != OGRERR_NONE) {
                std::cerr << "[SpatialIndex] Warning: Failed to draw mesh debug box.\n";
            }
            OGRFeature::DestroyFeature(feat);
        }
        layer_mesh->SyncToDisk();
        GDALClose(ds_mesh);
    }

    // EXACT DISTANCE LINES TO THE MARGIN (Only for points in the navigable area)
    if (!rtree_margin_.empty() && !test_points.empty()) {
        std::string path_line = output_folder + "/96_RTree_Distancia.shp";
        if (std::filesystem::exists(path_line)) driver->Delete(path_line.c_str());
        GDALDataset* ds_line = driver->Create(path_line.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        OGRLayer* layer_line = ds_line->CreateLayer("linhas", &srs, wkbLineString, nullptr);
        
        OGRFieldDefn field_type("Tipo", OFTString); // Type field
        if (layer_line->CreateField(&field_type) != OGRERR_NONE) {
            std::cerr << "[SpatialIndex] Warning: Failed to create Type field.\n";
        }
        OGRFieldDefn field_dist("Distancia", OFTReal); // Distance field
        if (layer_line->CreateField(&field_dist) != OGRERR_NONE) {
            std::cerr << "[SpatialIndex] Warning: Failed to create Distance field.\n";
        }

        for (const auto& pt : test_points) {
            BgPoint search_point(pt.get_x(), pt.get_y());

            // Validates if the point is in the navigable area before drawing the distance line
            bool is_navigable_point = false;
            std::vector<RTreePolygonValue> candidates;
            rtree_mesh_.query(bgi::intersects(search_point), std::back_inserter(candidates));
            for (const auto& candidate : candidates) {
                if (bg::within(search_point, mesh_triangles_[candidate.second])) {
                    is_navigable_point = true;
                    break;
                }
            }

            if (!is_navigable_point) continue; // Skips points outside the navigable area

            std::vector<RTreeSegmentValue> results;
            rtree_margin_.query(bgi::nearest(search_point, 50), std::back_inserter(results));
            
            if (!results.empty()) {
                BgSegment closest_seg;
                double min_dist = std::numeric_limits<double>::max();
                
                for (const auto& res : results) {
                    double d = bg::distance(search_point, res.second);
                    if (d < min_dist) {
                        min_dist = d;
                        closest_seg = res.second;
                    }
                }

                double px = search_point.get<0>(), py = search_point.get<1>();
                double ax = closest_seg.first.get<0>(), ay = closest_seg.first.get<1>();
                double bx = closest_seg.second.get<0>(), by = closest_seg.second.get<1>();

                double ab_x = bx - ax, ab_y = by - ay;
                double ap_x = px - ax, ap_y = py - ay;
                double dot_ab_ab = ab_x * ab_x + ab_y * ab_y;
                double t = (dot_ab_ab > 0.0) ? (ap_x * ab_x + ap_y * ab_y) / dot_ab_ab : 0.0;
                
                double proj_x = (t <= 0.0) ? ax : (t >= 1.0) ? bx : ax + t * ab_x;
                double proj_y = (t <= 0.0) ? ay : (t >= 1.0) ? by : ay + t * ab_y;

                OGRLineString distance_line;
                distance_line.addPoint(px, py); 
                distance_line.addPoint(proj_x, proj_y);
                
                OGRFeature* feat_dist = OGRFeature::CreateFeature(layer_line->GetLayerDefn());
                feat_dist->SetField("Tipo", "Distance_To_Margin"); // Translated field content
                feat_dist->SetField("Distancia", min_dist);
                feat_dist->SetGeometry(&distance_line);
                if (layer_line->CreateFeature(feat_dist) != OGRERR_NONE) {
                    std::cerr << "[SpatialIndex] Warning: Failed to draw distance line.\n";
                }
                OGRFeature::DestroyFeature(feat_dist);
            }
        }
        layer_line->SyncToDisk();
        GDALClose(ds_line);
    }
    std::cout << "[DEBUG] R-Trees and Distance Lines Shapefiles saved (Files 96, 97, and 98).\n";
}

// INTERFACE METHODS

/**
 * @brief Calculates the Euclidean distance to the nearest static obstacle.
 * @param position Current position of the vehicle (containing X and Y coordinates).
 * @return Distance in meters to the nearest mapped margin or obstacle.
 */
float SpatialIndex::get_closest_static_obstacle_distance(const types::Point& position) const {
    return static_cast<float>(calculate_margin_distance(position));
}

/**
 * @brief Checks if a given position is inside a restricted zone (outside the NavMesh).
 * @param position Current position of the vehicle to be tested.
 * @return true if the point is in a restricted/dangerous zone, false if it is in a safe navigable area.
 */
bool SpatialIndex::is_inside_restricted_zone(const types::Point& position) const {
    return !is_navigable(position);
}

/**
 * @brief Updates the internal cache of global targets monitored by the system.
 * @param targets Vector containing the new targets processed by the perception layer.
 */
void SpatialIndex::update_global_targets(const std::vector<types::Target>& targets) {
    global_targets_cache_ = targets;
}

/**
 * @brief Filters and returns active targets within a local range radius relative to a center.
 * @param center Central reference point.
 * @param radius Maximum search radius in meters.
 * @return Vector containing only the targets located within the area of interest.
 */
std::vector<types::Target> SpatialIndex::get_active_local_targets(const types::Point& center, float radius) const {
    std::vector<types::Target> local_targets;
    
    for (const auto& target : global_targets_cache_) {
        double dx = target.get_pose().get_x() - center.get_x();
        double dy = target.get_pose().get_y() - center.get_y();
        double dist = std::sqrt(dx * dx + dy * dy);
        
        if (dist <= radius) {
            local_targets.push_back(target);
        }
    }
    return local_targets;
}