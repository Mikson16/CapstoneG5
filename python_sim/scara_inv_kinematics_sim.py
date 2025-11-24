import numpy as np
import matplotlib.pyplot as plt
import time

class SCARA_Visualizer:
    def _init_(self, L1, L2):
        self.L1 = L1
        self.L2 = L2
        
        # Estado actual del robot (inicia en reposo sobre el eje X)
        self.q1_curr = 0.0
        self.q2_curr = 0.0 # Brazo estirado o plegado según tu configuración inicial
        
        # Configuración del gráfico
        plt.ion() # Activar modo interactivo
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.setup_plot()
        
        # Elementos gráficos del robot (líneas y puntos)
        # Link 1 (Base -> Codo)
        self.link1_line, = self.ax.plot([], [], 'o-', lw=5, color='#3498db', label='Eslabón 1')
        # Link 2 (Codo -> Muñeca)
        self.link2_line, = self.ax.plot([], [], 'o-', lw=4, color='#e74c3c', label='Eslabón 2')
        # Texto de información
        self.info_text = self.ax.text(0.05, 0.95, '', transform=self.ax.transAxes, fontsize=10,
                                      verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        # Dibujar robot en posición inicial
        self.update_robot_graphic(self.q1_curr, self.q2_curr)
        plt.show()

    def setup_plot(self):
        """Configura los límites y estética del gráfico"""
        max_reach = self.L1 + self.L2 + 0.5
        self.ax.set_xlim(-max_reach, max_reach)
        self.ax.set_ylim(-max_reach, max_reach)
        self.ax.set_aspect('equal')
        self.ax.grid(True, linestyle='--', alpha=0.6)
        self.ax.set_title("Simulación Interactiva SCARA 2-GDL")
        self.ax.set_xlabel("X [metros]")
        self.ax.set_ylabel("Y [metros]")
        
        # Dibujar área de trabajo (círculo máximo)
        circle = plt.Circle((0, 0), self.L1 + self.L2, color='gray', fill=False, linestyle=':', label='Alcance Máximo')
        self.ax.add_patch(circle)
        
        # Base del robot
        self.ax.plot(0, 0, 'ks', markersize=10, label='Base')
        self.ax.legend(loc='lower right')

    def inverse_kinematics(self, x, y):
        """
        TU CÓDIGO DE CINEMÁTICA INVERSA.
        Calcula q1 y q2 dados x, y.
        """
        h = np.sqrt(x*2 + y*2)
        
        # Verificación de alcance
        if h > (self.L1 + self.L2) or h < np.abs(self.L1 - self.L2):
            return None, None # Fuera de alcance

        # --- CÁLCULO (Basado en la corrección anterior) ---
        
        # 1. q2 (Ángulo de la articulación del codo)
        # Ley de cosenos para el ángulo interno
        cos_q2_int = (self.L1*2 + self.L22 - h*2) / (2 * self.L1 * self.L2)
        cos_q2_int = np.clip(cos_q2_int, -1.0, 1.0)
        q2_interno = np.arccos(cos_q2_int)
        
        # q2 real (suplementario para configuración estándar)
        q2 = np.pi - q2_interno

        # 2. q1 (Ángulo de la base)
        gamma = np.arctan2(y, x)
        
        # Ley de cosenos para beta
        if h == 0: return None, None # Singularidad en el origen
        
        cos_beta = (self.L1*2 + h2 - self.L2*2) / (2 * self.L1 * h)
        cos_beta = np.clip(cos_beta, -1.0, 1.0)
        beta = np.arccos(cos_beta)
        
        q1 = gamma - beta
        
        return q1, q2

    def update_robot_graphic(self, q1, q2):
        """Dibuja el robot usando Cinemática Directa para visualizar"""
        # Posición del codo (x1, y1)
        x1 = self.L1 * np.cos(q1)
        y1 = self.L1 * np.sin(q1)
        
        # Posición final (x2, y2)
        # Nota: El ángulo absoluto del eslabón 2 es (q1 + q2)
        x2 = x1 + self.L2 * np.cos(q1 + q2)
        y2 = y1 + self.L2 * np.sin(q1 + q2)
        
        # Actualizar líneas
        self.link1_line.set_data([0, x1], [0, y1])
        self.link2_line.set_data([x1, x2], [y1, y2])
        
        # Actualizar texto
        info = f"Coordenadas Efector:\nX: {x2:.3f}\nY: {y2:.3f}\n\nÁngulos:\nq1: {np.degrees(q1):.2f}°\nq2: {np.degrees(q2):.2f}°"
        self.info_text.set_text(info)

    def move_to(self, target_x, target_y, duration=1.0):
        """
        Anima el movimiento hacia el objetivo interpolando los ángulos.
        """
        # 1. Calcular ángulos objetivo (Inverse Kinematics)
        q1_target, q2_target = self.inverse_kinematics(target_x, target_y)
        
        if q1_target is None:
            print(f"⚠ Error: El punto ({target_x}, {target_y}) está fuera del alcance.")
            return

        print(f"✅ Moviendo a ({target_x}, {target_y})...")
        print(f"   Ángulos objetivo: q1={np.degrees(q1_target):.2f}°, q2={np.degrees(q2_target):.2f}°")

        # 2. Interpolación (Generar pasos suaves entre angulo actual y target)
        steps = 50 # Cantidad de cuadros de animación
        
        # Manejo de giros largos (elegir el camino más corto)
        # Esto es opcional pero ayuda a que el robot no de una vuelta de 360 innecesaria
        # (Aquí usamos interpolación lineal simple)
        
        q1_trajectory = np.linspace(self.q1_curr, q1_target, steps)
        q2_trajectory = np.linspace(self.q2_curr, q2_target, steps)
        
        # 3. Bucle de animación
        for i in range(steps):
            q1 = q1_trajectory[i]
            q2 = q2_trajectory[i]
            
            self.update_robot_graphic(q1, q2)
            
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(duration / steps)
            
        # 4. Actualizar estado interno al final
        self.q1_curr = q1_target
        self.q2_curr = q2_target
        print("   Movimiento completado.\n")

# --- BLOQUE PRINCIPAL ---
if _name_ == "_main_":
    # 1. Configuración del Robot
    l1_input = 1.0
    l2_input = 0.8
    
    sim = SCARA_Visualizer(l1_input, l2_input)
    
    print("-" * 50)
    print(f" SIMULADOR INTERACTIVO SCARA (L1={l1_input}, L2={l2_input})")
    print("-" * 50)
    print("Ingrese coordenadas X e Y para mover el robot.")
    print("Escriba 'salir' para terminar.")

    while True:
        try:
            user_input = input(">> Ingrese coordenada X, Y (ej: 0.5, 0.5): ")
            
            if user_input.lower() in ['salir', 'exit', 'q']:
                print("Cerrando simulación...")
                break
            
            # Parsear entrada
            parts = user_input.split(',')
            if len(parts) != 2:
                print("⚠ Formato incorrecto. Use: x, y")
                continue
                
            x_target = float(parts[0])
            y_target = float(parts[1])
            
            # Ejecutar movimiento
            sim.move_to(x_target, y_target)
            
        except ValueError:
            print("⚠ Error: Ingrese números válidos.")
        except Exception as e:
            print(f"⚠ Error inesperado: {e}")

    plt.close()
