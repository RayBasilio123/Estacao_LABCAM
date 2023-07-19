# Estação Meteorológica Inteligente

Este projeto tem como objetivo a construção de uma Estação Meteorológica Inteligente, equipada com um sistema redundante de coleta de dados, capaz de armazenar informações tanto localmente quanto em um banco de dados seguro, tornando-o aplicável em ambientes remotos e altamente confiável.

## Funcionamento Geral

A estação é composta por dois microcontroladores em comunicação ponto a ponto. Um dos microcontroladores fica instalado na estação, sendo responsável pelo processamento e transmissão dos dados coletados pelos sensores. O outro microcontrolador permanece no laboratorio, encarregado de receber os dados enviados pela estação.

A transmissão dos dados ocorre periodicamente por meio de uma conexão sem fio entre a estação e o receptor. Esse receptor, além de receber os pacotes de dados, realiza o encaminhamento das mensagens recebidas para o banco de dados MariaDB e para um sistema supervisório desenvolvido no Node-RED.

## Recursos Adicionais

- Cálculo de Eto (Evapotranspiração de Referência) - ✔️
- Utilização de modelos de aprendizado de máquina para previsões meteorológicas - 🚧
- Monitoramento contínuo dos dados coletados - ✔️

![Funcionamento Geral](funcionamento_geral.png)

Sinta-se à vontade para contribuir com melhorias e feedbacks neste projeto! Qualquer ajuda é bem-vinda.
