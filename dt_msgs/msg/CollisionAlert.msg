# Identificador MMSI do alvo móvel associado ao alerta.
# Deve corresponder ao campo mmsi de AisTarget.msg.
uint32 mmsi

# Indica se existe risco de colisão iminente com o alvo.
bool collision_imminent

# Tempo estimado até a colisão, em segundos.
# Valor negativo indica que a informação não está disponível.
float64 time_to_collision

# Distância mínima estimada entre o USV e o alvo, em metros.
# Valor negativo indica que a informação não está disponível.
float64 closest_approach_distance