#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>
#include "dt_core/types.hpp" 

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// Definições de primitivas geométricas utilizando o motor Boost.Geometry
typedef bg::model::d2::point_xy<double> BgPonto;
typedef bg::model::segment<BgPonto> BgSegmento;
typedef bg::model::box<BgPonto> BgCaixaReal;
typedef bg::model::polygon<BgPonto> BgPoligono;

// Pares indexados para inserção eficiente nas estruturas R-Tree do Boost
typedef std::pair<BgCaixaReal, BgSegmento> ValorRTreeSegmento;
typedef std::pair<BgCaixaReal, size_t> ValorRTreePoligono;

/**
 * @brief Motor de indexação e consultas espaciais em alta performance.
 * 
 * Utiliza árvores espaciais R-Tree (via Boost.Geometry) para realizar buscas de 
 * proximidade em tempo real (OLogN), verificação de zonas restritas (NavMesh) 
 * e gerenciamento de alvos dinâmicos no ecossistema do robô.
 */
class IndiceEspacial {
private:
    bgi::rtree<ValorRTreeSegmento, bgi::quadratic<16>> rtree_margem_; ///< Índice espacial R-Tree para os segmentos da margem de segurança.
    bgi::rtree<ValorRTreePoligono, bgi::quadratic<16>> rtree_malha_;  ///< Índice espacial R-Tree para os polígonos triangulados da NavMesh.
    std::vector<BgPoligono> triangulos_malha_;                       ///< Cache de geometria dos triângulos para consultas de inclusão.
    std::vector<types::Target> alvos_globais_cache_;                 ///< Cache interno para gerenciar o estado dos alvos dinâmicos.

public:
    /**
     * @brief Construtor padrão da classe IndiceEspacial.
     */
    IndiceEspacial() = default;

    /**
     * @brief Destrutor padrão da classe IndiceEspacial.
     */
    ~IndiceEspacial() = default;

    /**
     * @brief Carrega, processa e constrói as R-Trees espaciais a partir de arquivos Shapefile em disco.
     * @param shpMargem Caminho para o arquivo `.shp` referente à margem de segurança.
     * @param shpMalha Caminho para o arquivo `.shp` referente à malha de navegação.
     * @return true se o carregamento e indexação ocorreram com sucesso, false caso contrário.
     */
    bool carregarShapefiles(const std::string& shpMargem, const std::string& shpMalha);

    /**
     * @brief Calcula a distância euclidiana até o obstáculo estático mais próximo.
     * @param position Posição atual do veículo (contendo coordenadas X e Y).
     * @return Distância em metros até a margem ou obstáculo mapeado mais próximo.
     */
    float get_closest_static_obstacle_distance(const types::Point& position) const;

    /**
     * @brief Verifica se uma dada posição encontra-se dentro de uma zona restrita (fora da NavMesh).
     * @param position Posição atual do veículo a ser testada.
     * @return true se o ponto estiver em zona restrita/perigosa, false se estiver em área navegável segura.
     */
    bool is_inside_restricted_zone(const types::Point& position) const;
    
    /**
     * @brief Atualiza o cache interno de alvos globais monitorados pelo sistema.
     * @param targets Vetor contendo os novos alvos processados pela camada de percepção.
     */
    void update_global_targets(const std::vector<types::Target>& targets);

    /**
     * @brief Filtra e retorna os alvos ativos dentro de um raio de alcance local em relação a um centro.
     * @param center Ponto central de referência.
     * @param radius Raio máximo de busca em metros.
     * @return Vetor contendo apenas os alvos localizados dentro da área de interesse.
     */
    std::vector<types::Target> get_active_local_targets(const types::Point& center, float radius) const;

    // -------------------------------------------------------------------------
    // Métodos Base e Debug
    // -------------------------------------------------------------------------

    /**
     * @brief Método auxiliar que calcula a distância exata em metros até a margem de segurança.
     * @param posicao Posição de referência em formato `types::Point`.
     * @return Menor distância geométrica calculada via R-Tree.
     */
    double calcularDistanciaMargem(const types::Point& posicao) const;

    /**
     * @brief Avalia se um ponto específico pertence estritamente ao polígono navegável.
     * @param posicao Coordenadas de teste.
     * @return true se o ponto estiver contido na malha navegável, false caso contrário.
     */
    bool isNavegavel(const types::Point& posicao) const;

    /**
     * @brief Gera arquivos de depuração espacial para validação de consultas da R-Tree via QGIS.
     * @param pontos_teste Vetor de pontos de amostra para testes de proximidade.
     * @param pastaSaida Diretório de salvamento dos arquivos vetoriais gerados.
     * @param epsg_utm Código EPSG da projeção cartográfica UTM aplicada.
     */
    void exportarDebugRTree(const std::vector<types::Point>& pontos_teste, const std::string& pastaSaida, int epsg_utm) const;
};