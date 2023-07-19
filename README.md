Descrição do Projeto da Estação Meteorológica Utilizando LoRa e MQTT

Este repositório contém os arquivos desenvolvidos para a criação de uma estação meteorológica avançada, baseada na tecnologia LoRa (Long Range) e MQTT. A estação meteorológica é projetada para coletar e transmitir dados climáticos em um ambiente ponto a ponto, garantindo uma comunicação confiável e de longo alcance.

Principais Características:

Implementação de uma topologia ponto a ponto, permitindo a transmissão bidirecional de dados entre os dispositivos envolvidos no sistema.
Código-fonte completo e bem documentado do ESP que atua como o nó emissor, responsável por coletar informações meteorológicas a partir de sensores e transmiti-las por meio da rede LoRa.
Código-fonte detalhado do ESP que desempenha o papel de nó receptor, recebendo os dados LoRa e enviando-os para uma aplicação de monitoramento.
Arquivo exportado em formato JSON do Node-RED contendo a configuração dos nós do sistema, bem como a estilização personalizada para uma visualização intuitiva dos dados coletados.
Componentes do Projeto:

Arquivos ESP:

"esp_emissor.ino": Código do ESP nó emissor que coleta dados dos sensores e transmite-os através da rede LoRa.
"esp_receptor.ino": Código do ESP nó receptor que recebe os dados LoRa e os encaminha para a aplicação de monitoramento.
Arquivo JSON do Node-RED:

"configuracao_node_red.json": Arquivo JSON que contém a configuração dos nós do Node-RED, incluindo o tratamento dos dados recebidos e a interconexão com o protocolo MQTT.
"estilizacao_node_red.json": Arquivo JSON que contém a estilização personalizada da interface gráfica do Node-RED para visualização dos dados climáticos em tempo real.
