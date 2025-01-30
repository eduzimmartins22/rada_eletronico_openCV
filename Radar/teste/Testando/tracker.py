import cv2
import math
import time
import numpy as np
import os
from collections import defaultdict, deque
import serial
import serial.tools.list_ports

class Config:
   
    VELOCIDADE_LIMITE = 50  # km/h
    DISTANCIA_METROS = 20  
    
    # Configurações de detecção
    AREA_MINIMA = 8000
    ALTURA_MINIMA = 60
    LARGURA_MINIMA = 100
    PROPORCAO_MIN = 1.3
    PROPORCAO_MAX = 2.5
    
    # Configurações de ROI
    ROI_FINAL_MAX = 260
    ROI_FINAL_MIN = 200
    ROI_INICIO_MAX = 430
    ROI_INICIO_MIN = 500
    
    DISTANCIA_METROS = abs(ROI_INICIO_MAX - ROI_FINAL_MIN) * 0.1

  
    AREA_DETECCAO_SUPERIOR = 150  
    AREA_DETECCAO_INFERIOR = 550
    
    FPS = 30  
    PIXEL_TO_METERS = 0.05  
    MAX_TRACKING_AGE = 1.0 
    MIN_TRACKING_HITS = 3  
    
    @staticmethod
    def calcular_escala_pixels():
        """Calcula a escala de pixels por metro baseado na ROI"""
        pixels = abs(Config.ROI_INICIO_MAX - Config.ROI_FINAL_MIN)
        return pixels / Config.DISTANCIA_METROS


    @staticmethod
    def calcular_distancia_pixels():
        return abs(Config.ROI_INICIO_MAX - Config.ROI_FINAL_MIN)
    
    



class ControladorArquivos:
    def __init__(self):
        self.base_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Gravacao")
        self.exceeded_path = os.path.join(self.base_path, "excedido")
        self.captures_file = os.path.join(self.base_path, "capturas.txt")
        self.registros = []
        self._criar_diretorios()
        self._inicializar_arquivo_capturas()
        
    def _criar_diretorios(self):
        os.makedirs(self.base_path, exist_ok=True)
        os.makedirs(self.exceeded_path, exist_ok=True)
    
    def _inicializar_arquivo_capturas(self):
        try:
            with open(self.captures_file, "w") as f:
                f.write("ID\tVELOCIDADE\tDATA/HORA\n")
                f.write("-" * 50 + "\n")
            print(f"Arquivo de capturas inicializado em: {self.captures_file}")
        except Exception as e:
            print(f"Erro ao criar arquivo de capturas: {e}")
    
    def salvar_captura(self, imagem, id, velocidade, excedeu_limite):
        data_hora = time.strftime("%Y%m%d_%H%M%S")
        nome_arquivo = f"{data_hora}_ID{id}_velocidade_{velocidade:.1f}.jpg"
        
        caminho_arquivo = os.path.join(self.base_path, nome_arquivo)
        cv2.imwrite(caminho_arquivo, imagem)
        
        if excedeu_limite:
            caminho_excedido = os.path.join(self.exceeded_path, nome_arquivo)
            cv2.imwrite(caminho_excedido, imagem)
        
        # Log capture details
        with open(self.captures_file, "a") as f:
            linha = f"{data_hora}\tID: {id}\tVelocidade: {velocidade:.2f} km/h\t{'Excedeu Limite' if excedeu_limite else ''}\n"
            f.write(linha)

          
class RastreadorVeiculos:
    def __init__(self):
        self.centros = {}
        self.contador_id = 0
        self.tempos_entrada = {}
        self.tempos_saida = {}
        self.velocidades = {}
        self.posicoes_anteriores = {}
        self.vehicle_positions = defaultdict(list)
        self.pixel_to_meter_ratio = Config.PIXEL_TO_METERS
        self.total_veiculos = 0
        self.total_excedidos = 0
        self.coordinates = defaultdict(lambda: deque(maxlen=30))
        self.last_speed_update = {}
        self.speed_window = {}
        self.min_detection_count = 5

    def calculate_speed(self, vehicle_positions):
        if len(vehicle_positions) < self.min_detection_count:
            return 0
        
        first_pos = vehicle_positions[0]
        last_pos = vehicle_positions[-1]
        
        time_diff = last_pos[2] - first_pos[2]
        if time_diff <= 0:
            return 0
        
        dx = last_pos[0][0] - first_pos[0][0]
        dy = last_pos[0][1] - first_pos[0][1]
        distance_pixels = math.sqrt(dx*dx + dy*dy)
        
        distance_meters = distance_pixels * self.pixel_to_meter_ratio
        speed = (distance_meters / time_diff) * 3.6
        
        if speed < 0 or speed > 150:
            return 0
        
        return speed


    def atualizar(self, deteccoes):
        objetos_ids = []
        tempo_atual = time.time()

        for x, y, w, h in deteccoes:
            centro_x = (x + x + w) // 2
            centro_y = (y + y + h) // 2
            proporcao = float(w) / h

            if not (Config.PROPORCAO_MIN <= proporcao <= Config.PROPORCAO_MAX):
                continue
            if not (Config.ALTURA_MINIMA <= h and Config.LARGURA_MINIMA <= w):
                continue

            objeto_encontrado = False

            for id, centro in list(self.centros.items()):
                distancia = math.hypot(centro_x - centro[0], centro_y - centro[1])

                if distancia < 150:
                    centro_atual = (centro_x, centro_y)
                    bbox = (x, y, w, h)
                    self.vehicle_positions[id].append((centro_atual, bbox, tempo_atual))
                    self.coordinates[id].append(centro_y)

                    if len(self.vehicle_positions[id]) > 10:
                        self.vehicle_positions[id] = self.vehicle_positions[id][-10:]

                    if centro_y <= Config.ROI_INICIO_MAX and id not in self.tempos_entrada:
                        self.tempos_entrada[id] = tempo_atual
                    elif centro_y >= Config.ROI_FINAL_MIN and id not in self.tempos_saida:
                        self.tempos_saida[id] = tempo_atual

                    velocidade_atual = self.calculate_speed(self.vehicle_positions[id])
                    if 0 < velocidade_atual < 200:
                        self.velocidades[id] = velocidade_atual

                    self.centros[id] = (centro_x, centro_y)
                    objetos_ids.append([x, y, w, h, id, velocidade_atual])
                    objeto_encontrado = True
                    break

            if not objeto_encontrado:
                self.centros[self.contador_id] = (centro_x, centro_y)
                centro_atual = (centro_x, centro_y)
                bbox = (x, y, w, h)
                self.vehicle_positions[self.contador_id].append((centro_atual, bbox, tempo_atual))
                self.coordinates[self.contador_id].append(centro_y)
                self.tempos_entrada[self.contador_id] = tempo_atual
                objetos_ids.append([x, y, w, h, self.contador_id, 0])
                self.contador_id += 1

        for id in self.tempos_saida:
            if id in self.tempos_entrada:
                if id in self.velocidades and self.velocidades[id] > Config.VELOCIDADE_LIMITE:
                    self.total_excedidos += 1

        return objetos_ids



    def desenhar_info(self, frame, objetos_ids):
        for obj in objetos_ids:
            x, y, w, h, id, velocidade = obj
            cor = (0, 255, 0)
            
            if velocidade > 0:
                if velocidade > Config.VELOCIDADE_LIMITE:
                    cor = (0, 0, 255)
                else:
                    cor = (255, 0, 0)
            
            cv2.rectangle(frame, (x, y), (x + w, y + h), cor, 2)
            info_texto = f"ID: {id}"
            if velocidade > 0:
                info_texto += f" {velocidade:.2f}M/s"
            cv2.putText(frame, info_texto, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, cor, 2)

    def filter_detections(self, detections):
        if not detections:
            return []
        
        boxes = np.array(detections)
        areas = boxes[:, 2] * boxes[:, 3]
        order = np.argsort(areas)[::-1]
        
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            
            if order.size == 1:
                break
            
            xx1 = np.maximum(boxes[i, 0], boxes[order[1:], 0])
            yy1 = np.maximum(boxes[i, 1], boxes[order[1:], 1])
            xx2 = np.minimum(boxes[i, 0] + boxes[i, 2], 
                       boxes[order[1:], 0] + boxes[order[1:], 2])
            yy2 = np.minimum(boxes[i, 1] + boxes[i, 3], 
                       boxes[order[1:], 1] + boxes[order[1:], 3])
            
            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)
            intersection = w * h
            
            min_area = np.minimum(areas[i], areas[order[1:]])
            overlap_ratio = intersection / min_area
            
            inds = np.where(overlap_ratio <= 0.3)[0]
            order = order[inds + 1]
        
        return [detections[i] for i in keep]
            

class SerialHandler:
    def __init__(self):
        self.serial_port = None
        self.connected = False
        
    def scan_ports(self):
        """Scan for available serial ports and return a list of port names."""
        available_ports = []
        ports = serial.tools.list_ports.comports()
        
        for port in ports:
            available_ports.append(port.device)
        
        return available_ports
    
    def connect(self, port, baudrate=9600):
        """Try to connect to specified serial port."""
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.connected = True
            print(f"Connected to {port}")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to {port}: {e}")
            self.connected = False
            return False
            
    def send_speed_violation(self, id, speed):
        """Send speed violation data through serial port."""
        if not self.connected:
            return
            
        message = f"ID {id} velocidade {speed:.1f} km/h\n"
        try:
            self.serial_port.write(message.encode())
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
            self.connected = False
            
    def close(self):
        """Close serial connection."""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.connected = False






class ProcessadorVideo:
    def __init__(self, caminho_video):
        self.cap = cv2.VideoCapture(caminho_video)
        if not self.cap.isOpened():
            raise Exception("Erro ao abrir o vídeo")
            
        self.rastreador = RastreadorVeiculos()
        self.controlador_arquivos = ControladorArquivos()  
        self.detector = cv2.createBackgroundSubtractorMOG2(detectShadows=True)
        
        # Configurações de exibição
        self.largura_tela = 1280
        self.altura_tela = 720
        self.fps = 30
        self.configurar_visualizacao()
        
        # Kernels para operações morfológicas
        self.kernel_abertura = np.ones((3, 3), np.uint8)
        self.kernel_fechamento = np.ones((11, 11), np.uint8)
        self.kernel_erosao = np.ones((5, 5), np.uint8)
        
        self.ids_processados = set()
        self.flags_captura = {}
    
    def configurar_visualizacao(self):
        cv2.namedWindow("radar de velocidade", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("radar de velocidade", self.largura_tela, self.altura_tela)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
    
    def processar_frame(self, frame):
        altura, largura = frame.shape[:2]
        mascara_roi = np.zeros(frame.shape[:2], dtype=np.uint8)
        roi_superior = Config.AREA_DETECCAO_SUPERIOR
        roi_inferior = Config.AREA_DETECCAO_INFERIOR
        mascara_roi[roi_superior:roi_inferior, :] = 255

        mascara = self.detector.apply(frame)
        mascara = cv2.bitwise_and(mascara, mascara, mask=mascara_roi)

        # Adjust thresholding
        _, mascara_bin = cv2.threshold(mascara, 240, 255, cv2.THRESH_BINARY)
        
        # Refine morphological operations
        kernel_abertura = np.ones((2, 2), np.uint8)
        kernel_fechamento = np.ones((5, 5), np.uint8)
        
        mascara_limpa = cv2.morphologyEx(mascara_bin, cv2.MORPH_OPEN, kernel_abertura)
        mascara_limpa = cv2.morphologyEx(mascara_limpa, cv2.MORPH_CLOSE, kernel_fechamento)
        
        # Reduce erosion
        mascara_final = mascara_limpa

        cv2.imshow("Mask", mascara_final)
        
        contornos, _ = cv2.findContours(mascara_final, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        deteccoes = []

        for contorno in contornos:
            area = cv2.contourArea(contorno)
            if area > Config.AREA_MINIMA:
                x, y, w, h = cv2.boundingRect(contorno)
                if (roi_superior <= y <= roi_inferior and
                    w > Config.LARGURA_MINIMA and
                    h > Config.ALTURA_MINIMA and
                    Config.PROPORCAO_MIN <= float(w) / h <= Config.PROPORCAO_MAX):
                    deteccoes.append([x, y, w, h])

        return self.rastreador.filter_detections(deteccoes) if deteccoes else []
    
    def desenhar_roi(self, frame):
        altura, largura = frame.shape[:2]
        # Desenha linhas de ROI
        cv2.line(frame, (0, Config.ROI_INICIO_MAX), (largura, Config.ROI_INICIO_MAX), (0, 0, 255), 2)
        cv2.line(frame, (0, Config.ROI_INICIO_MIN), (largura, Config.ROI_INICIO_MIN), (0, 0, 255), 2)
        cv2.line(frame, (0, Config.ROI_FINAL_MAX), (largura, Config.ROI_FINAL_MAX), (0, 0, 255), 2)
        cv2.line(frame, (0, Config.ROI_FINAL_MIN), (largura, Config.ROI_FINAL_MIN), (0, 255, 255), 2)
        
        # Desenha área de detecção
        cv2.line(frame, (0, Config.AREA_DETECCAO_SUPERIOR), (largura, Config.AREA_DETECCAO_SUPERIOR), (255, 0, 0), 1)
        cv2.line(frame, (0, Config.AREA_DETECCAO_INFERIOR), (largura, Config.AREA_DETECCAO_INFERIOR), (255, 0, 0), 1)



        
    
    def executar(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
                
            altura, largura = frame.shape[:2]
            deteccoes = self.processar_frame(frame)
            objetos_rastreados = self.rastreador.atualizar(deteccoes)
            
            for obj in objetos_rastreados:
                x, y, w, h, id, velocidade = obj  
                if id in self.ids_processados:
                    continue
                    
                excedeu_limite = velocidade > Config.VELOCIDADE_LIMITE
                
                # Desenha retângulo e velocidade
                cor = (0, 0, 255) if excedeu_limite else (0, 255, 0)
                cv2.rectangle(frame, (x, y), (x + w, y + h), cor, 2)
                cv2.putText(frame, f"ID: {id} {velocidade:.1f}km/h", 
                           (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, cor, 2)
                
                # Captura imagem se necessário
                if velocidade > 0 and id not in self.flags_captura:
                    self.flags_captura[id] = True
                    imagem_captura = frame[y:y+h, x:x+w].copy()
                    self.controlador_arquivos.salvar_captura(
                        imagem_captura, id, velocidade, excedeu_limite
                    )
            
            self.desenhar_roi(frame)
            cv2.imshow("radar de velocidade", frame)
            
            if cv2.waitKey(1) & 0xFF == 27:  # ESC para sair
                break
        
        self.cap.release()
        cv2.destroyAllWindows()

def main():
    try:
        caminho_video = r"Radar\teste\Testando\video.mp4"
        processador = ProcessadorVideo(caminho_video)
        processador.executar()
    except Exception as e:
        print(f"Erro durante a execução: {e}")
        serial_handler = SerialHandler()
        available_ports = serial_handler.scan_ports()
    if not available_ports:
        print("No serial ports available")
        return

    print("Available ports:", available_ports)
    for port in available_ports:
        if serial_handler.connect(port):
            break

if __name__ == "__main__":
    main()