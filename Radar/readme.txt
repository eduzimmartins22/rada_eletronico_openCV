📡 Radar Eletrônico 🚗💨



🛠️ Sobre o Projeto

Este é um Radar Eletrônico baseado em visão computacional para monitoramento de velocidade de veículos. O sistema analisa um fluxo de vídeo, detecta veículos e calcula sua velocidade, registrando infrações em caso de excesso de velocidade.

🚀 Tecnologias Utilizadas

OpenCV - Processamento de vídeo e detecção de objetos

NumPy - Manipulação de arrays e cálculos matemáticos

Serial - Comunicação com dispositivos externos

Python - Linguagem de programação

📌 Funcionalidades

✅ Detecção de veículos em tempo real✅ Cálculo preciso de velocidade✅ Registro de infrações com salvamento de imagens✅ Comunicação com hardware via porta serial✅ Exibição gráfica das áreas de monitoramento

📷 Exemplo de Detecção

🔧 Configuração

Antes de rodar o sistema, instale as dependências:

pip install opencv-python numpy pyserial

Depois, execute o radar com:

python radar.py

📂 Estrutura do Projeto

📁 Radar_Eletronico
│── 📂 Gravacao
│── 📜 radar.py
│── 📜 README.md
│── 📜 requisitos.txt

⚠️ Limites de Velocidade

O sistema possui uma configuração padrão de 50 km/h como limite. Este valor pode ser alterado no arquivo Config.

🎯 Como Funciona

1️⃣ O vídeo é processado em tempo real.2️⃣ Veículos são detectados e rastreados.3️⃣ A velocidade é calculada com base na distância percorrida por quadro.4️⃣ Se um veículo exceder o limite, sua imagem é salva e a infração é registrada.

📡 Comunicação Serial

O radar pode se conectar a um microcontrolador via porta serial para alertas e registros adicionais.

📜 Licença

Este projeto está sob a MIT License. Sinta-se à vontade para modificar e compartilhar!

💡 Desenvolvido por Eduardo Martins Cardoso- 🚀 Open Source para a Comunidade!

