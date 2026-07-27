#include "dt_viz/shapefile_loader.hpp"

#include <gdal_priv.h>
#include <ogrsf_frmts.h>

#include <iostream>

namespace
{

/**
 * @brief Adiciona os pontos de uma linha OGR a um QPainterPath.
 */
void appendLineString(
  const OGRLineString * line,
  QPainterPath & path,
  bool close_path)
{
  if (line == nullptr || line->getNumPoints() == 0) {
    return;
  }

  // O eixo Y é invertido porque, no Qt, valores positivos
  // crescem para baixo na tela.
  path.moveTo(
    line->getX(0),
    -line->getY(0));

  for (int index = 1; index < line->getNumPoints(); ++index) {
    path.lineTo(
      line->getX(index),
      -line->getY(index));
  }

  if (close_path) {
    path.closeSubpath();
  }
}

/**
 * @brief Converte um polígono OGR, incluindo seus possíveis
 * anéis internos, para um QPainterPath.
 */
void appendPolygon(
  const OGRPolygon * polygon,
  QPainterPath & path)
{
  if (polygon == nullptr) {
    return;
  }

  appendLineString(
    polygon->getExteriorRing(),
    path,
    true);

  for (int index = 0;
    index < polygon->getNumInteriorRings();
    ++index)
  {
    appendLineString(
      polygon->getInteriorRing(index),
      path,
      true);
  }

  // Permite representar corretamente buracos internos.
  path.setFillRule(Qt::OddEvenFill);
}

/**
 * @brief Processa recursivamente os tipos de geometria
 * suportados pela interface.
 */
void appendGeometry(
  const OGRGeometry * geometry,
  QPainterPath & path)
{
  if (geometry == nullptr) {
    return;
  }

  const OGRwkbGeometryType type =
    wkbFlatten(geometry->getGeometryType());

  switch (type) {
    case wkbPolygon:
    {
      appendPolygon(
        geometry->toPolygon(),
        path);
      break;
    }

    case wkbMultiPolygon:
    {
      const auto * multi_polygon =
        geometry->toMultiPolygon();

      for (int index = 0;
        index < multi_polygon->getNumGeometries();
        ++index)
      {
        appendGeometry(
          multi_polygon->getGeometryRef(index),
          path);
      }

      break;
    }

    case wkbLineString:
    {
      appendLineString(
        geometry->toLineString(),
        path,
        false);
      break;
    }

    case wkbMultiLineString:
    {
      const auto * multi_line =
        geometry->toMultiLineString();

      for (int index = 0;
        index < multi_line->getNumGeometries();
        ++index)
      {
        appendGeometry(
          multi_line->getGeometryRef(index),
          path);
      }

      break;
    }

    case wkbGeometryCollection:
    {
      const auto * collection =
        geometry->toGeometryCollection();

      for (int index = 0;
        index < collection->getNumGeometries();
        ++index)
      {
        appendGeometry(
          collection->getGeometryRef(index),
          path);
      }

      break;
    }

    default:
    {
      // Pontos e outros tipos não são usados na carta atual.
      break;
    }
  }
}

}  // namespace

std::vector<ShapefileGeometry> ShapefileLoader::load(
  const std::string & file_path)
{
  std::vector<ShapefileGeometry> result;

  GDALAllRegister();

  auto * dataset = static_cast<GDALDataset *>(
    GDALOpenEx(
      file_path.c_str(),
      GDAL_OF_VECTOR | GDAL_OF_READONLY,
      nullptr,
      nullptr,
      nullptr));

  if (dataset == nullptr) {
    std::cerr
      << "[dt_viz] Não foi possível abrir o Shapefile: "
      << file_path
      << '\n';

    return result;
  }

  for (int layer_index = 0;
    layer_index < dataset->GetLayerCount();
    ++layer_index)
  {
    OGRLayer * layer =
      dataset->GetLayer(layer_index);

    if (layer == nullptr) {
      continue;
    }

    layer->ResetReading();

    OGRFeature * feature = nullptr;

    while ((feature = layer->GetNextFeature()) != nullptr) {
      const OGRGeometry * geometry =
        feature->GetGeometryRef();

      if (geometry != nullptr) {
        QPainterPath path;

        appendGeometry(
          geometry,
          path);

        if (!path.isEmpty()) {
          ShapefileGeometry converted;
          converted.path = path;
          converted.bounds = path.boundingRect();

          result.push_back(converted);
        }
      }

      OGRFeature::DestroyFeature(feature);
    }
  }

  GDALClose(dataset);

  return result;
}