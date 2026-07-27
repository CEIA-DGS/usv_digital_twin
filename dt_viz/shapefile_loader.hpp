#ifndef DT_VIZ_SHAPEFILE_LOADER_HPP
#define DT_VIZ_SHAPEFILE_LOADER_HPP

#include <QPainterPath>
#include <QRectF>

#include <string>
#include <vector>

/**
 * @brief Geometria vetorial carregada de um Shapefile.
 *
 * O caminho pode representar polígonos, multipolígonos,
 * linhas ou múltiplas linhas.
 */
struct ShapefileGeometry
{
  QPainterPath path;
  QRectF bounds;
};

/**
 * @brief Classe responsável por carregar arquivos Shapefile.
 *
 * A leitura é realizada por meio da biblioteca GDAL/OGR.
 */
class ShapefileLoader
{
public:
  /**
   * @brief Carrega todas as geometrias de um arquivo .shp.
   *
   * @param file_path Caminho completo do arquivo Shapefile.
   * @return Lista de geometrias convertidas para QPainterPath.
   */
  static std::vector<ShapefileGeometry> load(
    const std::string & file_path);
};

#endif