
from flask import Flask, render_template, request
import socket
from obj_def import OmniRobot, Obstacles, VisionData, Obstacle
from trajectory_functions import check_distances, Obstacle_Avoidance_Control, Trajectory_Control


app = Flask(__name__)

# Global object initialization for control
omni_robot = OmniRobot()
obstacles = Obstacles()
vision_data = VisionData()
ball = Obstacle(0, 0)

# UDP Configuration
# Replace with your actual UDP server IP and port
UDP_IP = "10.0.0.107"  #192.168.20.7
UDP_PORT = 12345           

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/send', methods=['POST'])
def send():
    command = request.form['command']
    # Expects command type: POS,x,y,angle
    if command.startswith('POS'):
        try:
            _, x, y, angle = command.split(',')
            # Update robot and ball state
            omni_robot.PosX = float(omni_robot.PosX)
            omni_robot.PosY = float(omni_robot.PosY)
            omni_robot.Angle = float(angle)
            ball.PosX = float(x)
            ball.PosY = float(y)
            # Process control logic
            is_ball_close, is_obstacle_close, obj_obstacle = check_distances(omni_robot, obstacles, ball)
            desired_position_omnirobot = [ball.PosX, ball.PosY]
            actual_position_omnirobot = [omni_robot.PosX, omni_robot.PosY]
            if is_obstacle_close:
                velocity_body_setpoints = Obstacle_Avoidance_Control(omni_robot, obj_obstacle)
            else:
                velocity_body_setpoints = Trajectory_Control(desired_position_omnirobot, actual_position_omnirobot)
            # Prepare the message to send (adjust the format as needed for your robot)
            msg = f"VEL,{velocity_body_setpoints[0]},{velocity_body_setpoints[1]},{velocity_body_setpoints[2]}"
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
            return f"Comando procesado y enviado: {msg}"
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
if __name__ == '__main__':
    print("Starting Flask server...")
    app.run(host='0.0.0.0', port=5000, debug=True)
    print("Flask server started at http://localhost:5000/")