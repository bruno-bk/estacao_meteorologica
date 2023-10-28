# Estação Meteorológica com ESP32

Este projeto consiste na criação de uma estação meteorológica utilizando a placa ESP32 e uma variedade de sensores para medir diferentes parâmetros climáticos. Os sensores utilizados incluem os seguintes:

- Sensor de pressão
- Sensor de temperatura
- Sensor de umidade
- Sensor de luminosidade
- Chave optica (velocidade do vento)
- Sensor de direção do vento

O desenvolvimento deste projeto será realizado no Visual Studio Code usando a extensão PlatformIO.

## Configuração do Hardware

- ESP32
- Sensor de pressão e temperatura [BMP280](https://www.mouser.com/datasheet/2/783/BST_BMP280_DS001-1509562.pdf)
- Sensor de umidade [DHT11](https://www.mouser.com/datasheet/2/758/DHT11-Technical-Data-Sheet-Translated-Version-1143054.pdf)
- Sensor de luminosidade [BH1750](https://www.mouser.com/datasheet/2/348/bh1750fvi-e-186247.pdf)
- Sensor de velocidade do vento [Encoder](https://www.mouser.com/datasheet/2/405/lm393-1487637.pdf)
- Sensor de direção do vento [AS5600](https://www.mouser.com/datasheet/2/588/AS5600_UG000254_2_00-1877354.pdf)

Certifique-se de conectar os sensores corretamente ao ESP32 de acordo com as instruções do fabricante.

## Configuração do Ambiente de Desenvolvimento

1. Instale o Visual Studio Code (VSCode).
2. Instale a extensão PlatformIO no VSCode.
3. Faça o clone desse projeto.
4. Altere os campos de wifi e broker MQTT no arquivo config.h.
4. Carregue o código fonte deste repositório no projeto PlatformIO.
5. Na primera vez que o projeto for compilado, será realizado o download das bibliotecas necessárias.

## Uso

Uma vez que o hardware e o software estejam configurados corretamente, o ESP32 começará a coletar dados meteorológicos automaticamente. Esses dados serão enviados ao broken MQTT configurado no código.

## Contribuição

Contribuições são bem-vindas! Sinta-se à vontade para fazer um fork deste repositório, trabalhar em melhorias e enviar um pull request.

## Licença

Este projeto está licenciado sob a GNU General Public License.

Para mais detalhes, consulte a [Licença](https://www.gnu.org/licenses/gpl-3.0.html).
