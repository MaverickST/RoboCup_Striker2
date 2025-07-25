from flask import Flask, render_template, request
import socket
# from obj_def import OmniRobot, Obstacles, VisionData, Obstacle
# from trajectory_functions import check_distances, Obstacle_Avoidance_Control, Trajectory_Control


app = Flask(__name__)

# Global object initialization for control
# omni_robot = OmniRobot()
# obstacles = Obstacles()
# vision_data = VisionData()
# ball = Obstacle(0, 0)

# Inicialización de variables
robot_info = [0.0, 0.0, 0.0]  # [PosX, PosY, Angle]
ball_info = [0.0, 0.0]        # [PosX, PosY]
obstacles_matrix = [[], []]   # 2 filas: [PosX_list, PosY_list], columnas = obstáculos

# UDP Configuration
# Replace with your actual UDP server IP and port
UDP_IP = "10.0.0.107"  #192.168.20.7
UDP_PORT = 12345           

@app.route('/')
def index():
    return render_template('index.html')

def send_velocity_to_robot(velocity_body_setpoints):
    """
    Envía los datos de velocidad al robot por UDP.
    Retorna un mensaje de éxito o error.
    """
    try:
        msg = f"VEL,{velocity_body_setpoints[0]},{velocity_body_setpoints[1]},{velocity_body_setpoints[2]}"
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
        return f"Comando procesado y enviado: {msg}"
    except Exception as e:
        return f"Error enviando velocidades: {str(e)}"

@app.route('/send', methods=['POST'])
def send():
    command = request.form['command']
    # Expects command type: POS,x,y,angle o trama extendida
    if command.startswith('POS'):
        try:
            # 
            # _, x, y, angle = command.split(',')
            # # Update robot and ball state
            # omni_robot.PosX = float(omni_robot.PosX)
            # omni_robot.PosY = float(omni_robot.PosY)
            # omni_robot.Angle = float(angle)
            # ball.PosX = float(x)
            # ball.PosY = float(y)
            # # Process control logic
            # is_ball_close, is_obstacle_close, obj_obstacle = check_distances(omni_robot, obstacles, ball)
            # desired_position_omnirobot = [ball.PosX, ball.PosY]
            # actual_position_omnirobot = [omni_robot.PosX, omni_robot.PosY]
            # if is_obstacle_close:
            #     velocity_body_setpoints = Obstacle_Avoidance_Control(omni_robot, obj_obstacle)
            # else:
            #     velocity_body_setpoints = Trajectory_Control(desired_position_omnirobot, actual_position_omnirobot) 
            #
            # Nueva lógica: escribir en variables globales
            global robot_info, ball_info, obstacles_matrix
            data = command.split(',')
            if len(data) == 4:
                # Solo robot y ball (ball = robot x,y)
                _, x, y, angle = data
                robot_info[0] = float(x)
                robot_info[1] = float(y)
                robot_info[2] = float(angle)
                ball_info[0] = float(x)
                ball_info[1] = float(y)
                # Obstáculos vacíos
                obstacles_matrix[0] = []
                obstacles_matrix[1] = []
            elif len(data) >= 5:
                # robot x,y,angle, ball x,y, [obs1x, obs1y, ...]
                robot_info[0] = float(data[0])
                robot_info[1] = float(data[1])
                robot_info[2] = float(data[2])
                ball_info[0] = float(data[3])
                ball_info[1] = float(data[4])
                obs_data = data[5:]
                if len(obs_data) % 2 != 0:
                    return "Error: La cantidad de datos de obstáculos debe ser par (x,y por cada obstáculo)."
                num_obs = len(obs_data) // 2
                obstacles_matrix[0] = [float(obs_data[i*2]) for i in range(num_obs)]
                obstacles_matrix[1] = [float(obs_data[i*2+1]) for i in range(num_obs)]
            else:
                return "Error: Se requieren al menos 4 valores para POS o 5+ para trama extendida."
            return (
                f"Datos escritos: robot_info={robot_info}, "
                f"ball_info={ball_info}, "
                f"obstacles_matrix={obstacles_matrix}"
            )
        except Exception as e:
            return f"Error procesando comando: {str(e)}"
    else:
        # If not a POS command, forward as is
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(command.encode(), (UDP_IP, UDP_PORT))
            return f"Command sent: {command}"
        except Exception as e:
            return f"Error sending command: {str(e)}"

def get_current_vision_data():
    """
    Retorna los datos actuales de robot, ball y obstacles.
    """
    return robot_info, ball_info, obstacles_matrix

if __name__ == '__main__':
    print("Starting Flask server...")
    app.run(host='0.0.0.0', port=5000, debug=True)
    print("Flask server started at http://localhost:5000/")