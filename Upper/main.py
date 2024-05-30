from serialBuffer import *
from render import *
import time


def main():
    try:
        ser = SerialLineBuffer("COM8")
        ser.start()

        while True:
            screen.fill(BLACK)
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    ser.close()
                    sys.exit()

            # Read data from serial port
            line = ser.readline_newest()
            data = bytes_to_doubles(line)
            if len(data) != 6:
                continue
            # print(data)

            # Render the arm
            pitch_L = data[0] / 180 * np.pi; roll_L = data[1] / 180 * np.pi; yaw_L = data[2] / 180 * np.pi
            pitch_H = data[3] / 180 * np.pi; roll_H = data[4] / 180 * np.pi; yaw_H = data[5] / 180 * np.pi
            
            # Text
            delta_pitch = "delta_pitch: " + str((pitch_H - pitch_L) * 180 / np.pi)
            delta_roll = "delta_roll: " + str((roll_H - roll_L) * 180 / np.pi)
            delta_yaw = "delta_yaw: " + str((yaw_H - yaw_L) * 180 / np.pi) 
            print(delta_pitch, delta_roll, delta_yaw)
            

            # 画文字
            draw_text(screen, delta_pitch, (100, 10), GREEN)
            draw_text(screen, delta_roll, (100, 50), BLUE)
            draw_text(screen, delta_yaw, (100, 90), LIGHT_BLUE)

            # 计算旋转矩阵
            R_upper = rotation_matrix(pitch_H, roll_H, yaw_H)
            R_lower = rotation_matrix(pitch_L, roll_L, yaw_L)  
            
            # 旋转长方体
            rotated_upper_arm = [np.dot(R_upper, point) for point in upper_arm]
            rotated_lower_arm = [np.dot(R_lower, point) for point in lower_arm]
            
            # 平移长方体
            transformed_upper_arm = [point + upper_arm_position for point in rotated_upper_arm]
            transformed_lower_arm = [point + lower_arm_position for point in rotated_lower_arm]
            
            # 画长方体
            draw_cube(transformed_upper_arm)
            draw_cube(transformed_lower_arm)
            
            pygame.display.flip()

            
            

    except KeyboardInterrupt:
        if ser:
            print("Closing serial port")
            ser.close()

if __name__ == "__main__":
    main()


