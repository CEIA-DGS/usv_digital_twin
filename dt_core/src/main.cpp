#include <iostream>
#include <string>
#include <filesystem>
#include "GdalInicializador.hpp"
#include "ProcessadorS57.h"
#include "GeradorMalha.h"
#include "ExportadorVetor.h"
#include "VisualizadorVetor.h"
#include "IndiceEspacial.hpp"
#include "Types.hpp"
#include "gdal_priv.h"

namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
    try {
        if (argc < 2) {
            std::cerr << "Uso correto: " << argv[0] << " <caminho_para_carta.039>\n";
            return 1;
        }

        std::string caminhoCartaS57 = argv[1];
        std::string nomeCarta = fs::path(caminhoCartaS57).stem().string();
        std::string diretorioSaida = "../data/output/NavMesh_Shapefiles_" + nomeCarta;

        // Inicialização global
        GdalInicializador::inicializar();
        std::cout << "[Sistema] Processando carta náutica: " << caminhoCartaS57 << std::endl;

        // Processamento S-57
        ConfiguracaoMapa config = GerenciadorConfig::carregarConfiguracao("../config.json");
        GeometriasProcessadas geometrias = ProcessadorS57::processarCarta(caminhoCartaS57, config);
        
        // Extrai o EPSG calculado da estrutura processada
        int epsg_utm = geometrias.epsg_utm;

        // Geração da NavMesh
        MalhaNavegacao navMesh = GeradorMalha::gerar(geometrias, config.margemSeguranca, config.toleranciaSimplificacao);

        // Exportação e Visualização
        if (fs::exists(diretorioSaida)) fs::remove_all(diretorioSaida);
        ExportadorVetor::exportarShapefile(navMesh, diretorioSaida, epsg_utm);
        VisualizadorVetor::exibir(diretorioSaida, nomeCarta);

        // Carregamento do Motor Espacial para Testes
        IndiceEspacial motorEspacial;
        motorEspacial.carregarShapefiles(diretorioSaida + "/2_Margem_Seguranca.shp", diretorioSaida + "/4_Malha_NavMesh.shp");

        // Teste rápido utilizando o centro geométrico da malha
        GDALDataset* ds_malha = (GDALDataset*)GDALOpenEx((diretorioSaida + "/4_Malha_NavMesh.shp").c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
        OGREnvelope env_malha;
        if (ds_malha) {
            (void)ds_malha->GetLayer(0)->GetExtent(&env_malha);
            GDALClose(ds_malha);
        }
        
        types::Point pontoTeste((env_malha.MinX + env_malha.MaxX) / 2.0, (env_malha.MinY + env_malha.MaxY) / 2.0, 0.0);
        
        std::cout << "\n-> Distância ao obstaculo estatico: " << motorEspacial.get_closest_static_obstacle_distance(pontoTeste) << " m.\n";
        std::cout << "-> O ponto esta em zona restrita? " << (motorEspacial.is_inside_restricted_zone(pontoTeste) ? "SIM" : "NAO") << "\n";

        // Teste de alvos dinâmicos / barcos próximos dentro de um raio de alcance (ex: 500m)
        std::vector<types::Target> alvosSimulados = {
            types::Target(1, "Barco_Proximo (30m)",    types::Pose(pontoTeste.get_x() + 20.0,  pontoTeste.get_y() + 20.0,  0.0), types::Kinematics(types::Velocity())),
            types::Target(2, "Barco_Medio (250m)",     types::Pose(pontoTeste.get_x() + 200.0, pontoTeste.get_y() + 150.0, 0.0), types::Kinematics(types::Velocity())),
            types::Target(3, "Barco_Distante (1.2km)", types::Pose(pontoTeste.get_x() + 1000.0, pontoTeste.get_y() + 800.0, 0.0), types::Kinematics(types::Velocity()))
        };
        motorEspacial.update_global_targets(alvosSimulados);

        float raioBuscaMetros = 500.0f;
        auto alvosLocais = motorEspacial.get_active_local_targets(pontoTeste, raioBuscaMetros);
        
        std::cout << "-> Alvos detectados no raio de " << raioBuscaMetros << "m: " << alvosLocais.size() << "\n";
        for (const auto& alvo : alvosLocais) {
            std::cout << "   * [Alvo Local] " << alvo.get_description() << " (ID: " << alvo.get_id() << ")\n";
        }

        geometrias.liberarMemoria();
        navMesh.liberarMemoria();

        std::cout << "\n[Sistema] Execucao finalizada com sucesso!\n";
    } catch (const std::exception& e) {
        std::cerr << "\n[Erro Crítico] " << e.what() << std::endl;
        return 1;
    }
    return 0;
}