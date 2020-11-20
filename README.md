# Firmware

## Configuração 

Algumas variáveis já estão pré-configuradas no arquivo `sdkconfig.defaults`. Para utilizar essas configurações, use o comando `idf.py reconfigure` que o arquivo `sdkconfig` será criado a partir do default. 

Além disso, o projeto precisa de mais algumas configurações. Utilize `idf.py menuconfig` e configure as seguintes variáveis:

### **Main Firmware Configuration**
### Device ID

> **CONFIG_DEVICE_ID**: Define o nome do device na rede, utilizado para identificá-lo no dashboard. É enviado no começo de todo conjunto de dados. Precisa ser uma **string de 5 caracteres**, com valor default de `esp_1`. 

### MQTT Connection Parameters

> **CONFIG_BROKER_URL**: URL do MQTT Broker para onde as mensagens serão enviadas.

> **CONFIG_BROKER_USERNAME**: Username utilizado para autenticação com o Broker.

> **CONFIG_BROKER_PASSWORD**: Senha utilizada para autenticação com o Broker.

### **BLE Mesh Custom Sensor Model Configuration**

Pode ser acessado dentro do menu "Components", configura o tipo de device na rede mesh (gateway ou sensor).

> **MESH_DEVICE_TYPE**: Deve ser escolhido entre `CONFIG_MESH_DEVICE_TYPE_GATEWAY` e `CONFIG_COENV_NODE_TYPE_SENSOR`.

> **MESH_DEVICE_NAME**: String que define o nome que será utilizado pelo Bluetooth para advertising. Utilizado apenas para encontrar o dispositivo quando ele estiver desprovisionado.

### **Example Connection Configuration**

Referente às configurações de Wi-Fi. Os campos `CONFIG_EXAMPLE_WIFI_SSID` e `CONFIG_EXAMPLE_WIFI_PASSWORD` devem ser editados com os dados da rede Wi-Fi utilizada para conexão. Essa configuração precisa ser feita apenas para o *Gateway*. 


## Provisionamento

:construction:
