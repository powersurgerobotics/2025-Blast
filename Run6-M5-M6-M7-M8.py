from hub import light_matrix, port, motion_sensor
import motor, motor_pair
import color_sensor
import runloop
import time


max_speed = 1050

drive_motors = motor_pair.pair(motor_pair.PAIR_1, port.C, port.D)
attachment_motors = motor_pair.pair(motor_pair.PAIR_2, port.A, port.B)

async def Gyro_Right_turn(degrees_to_turn):
    motion_sensor.set_yaw_face(motion_sensor.RIGHT)
    motion_sensor.reset_yaw(0) #reset yaw angle
    degrees_to_turn_final = -1*degrees_to_turn*10
    print(degrees_to_turn_final)
    while motion_sensor.tilt_angles()[0]>degrees_to_turn_final:
        motor_pair.move(motor_pair.PAIR_1,100)
    motion_sensor.reset_yaw(0) #reset yaw angle
    motor_pair.stop(motor_pair.PAIR_1)

async def Gyro_Left_turn(degrees_to_turn):
    motion_sensor.set_yaw_face(motion_sensor.RIGHT)
    motion_sensor.reset_yaw(0) #reset yaw angle
    degrees_to_turn_final = degrees_to_turn*10
    print(degrees_to_turn_final)
    while motion_sensor.tilt_angles()[0]<degrees_to_turn_final:
        motor_pair.move(motor_pair.PAIR_1,-100)
    motion_sensor.reset_yaw(0) #reset yaw angle
    motor_pair.stop(motor_pair.PAIR_1)

async def Gyro_Move_Forward_By_Distance(Yaw, Distance, Speed_pct):
    # assumptions: motion_sensor has a yaw() method; left/right motor ports are known
    motion_sensor.reset_yaw(0)
    Speed = int(Speed_pct * max_speed / 100)
    # reset both drive encoders if possible (left & right)
    motor.reset_relative_position(port.D, 0)
    wheel_circumference = 6.9 # 176mm, 17.6cm, 6.92 inches
    if wheel_circumference == 0:
        raise ValueError("wheel_circumference must be nonzero")
    Distance_Degrees =round(Distance * (360.0 / wheel_circumference))
    #print(" Distance "+ str(Distance_Degrees))
    p = 0.5
    runloop.sleep_ms(500)
    start_time = time.time()
    timeout = 10# seconds, safety timeout

    while abs(motor.relative_position(port.D)) <= Distance_Degrees:
        # use dedicated yaw if available
        current_yaw =motion_sensor.tilt_angles()[0] #Current yaw value
        error = (Yaw - current_yaw)
        Correction = p * error
        left_motor_speed = int(max(-max_speed, min(max_speed, Speed - Correction)))
        right_motor_speed = int(max(-max_speed, min(max_speed, Speed + Correction)))
        motor_pair.move_tank(motor_pair.PAIR_1, left_motor_speed, right_motor_speed)
        runloop.sleep_ms(10)# let sensors update, avoid busy spin
        if time.time() - start_time > timeout:
            break# safety: prevent infinite loop
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)
    #print("Drive completed")

async def Gyro_Move_Backward_By_Distance(initial_yaw, Distance, Speed_pct):
    # assumptions: motion_sensor has a yaw() method; left/right motor ports are known
    motion_sensor.reset_yaw(0)
    Speed = int(Speed_pct * max_speed / 100)
    # reset both drive encoders if possible (left & right)
    motor.reset_relative_position(port.D, 0)
    wheel_circumference = 6.9 # 176mm, 17.6cm, 6.92 inches
    if wheel_circumference == 0:
        raise ValueError("wheel_circumference must be nonzero")
    Distance_Degrees =round(Distance * (360.0 / wheel_circumference))
    #print(" Distance "+ str(Distance_Degrees))
    p = 0.5
    runloop.sleep_ms(500)
    start_time = time.time()
    timeout = 10# seconds, safety timeout
    print("motor position " + str(motor.relative_position(port.D)))
    print("Distance in Degrees : " + str(Distance_Degrees ))
    #left_motor_speed =500#int(max(-max_speed, min(max_speed, Speed - Correction)))
    #right_motor_speed =500#int(max(-max_speed, min(max_speed, Speed + Correction)))
    while abs(motor.relative_position(port.D)) <= Distance_Degrees:
        # use dedicated yaw if available
        current_yaw =motion_sensor.tilt_angles()[0] #Current yaw value
        error = (initial_yaw - current_yaw)
        #Correction = p * error
        motor_pair.move_tank(motor_pair.PAIR_1, -1*Speed, -1*Speed)
        runloop.sleep_ms(10)# let sensors update, avoid busy spin
        if time.time() - start_time > timeout:
            break# safety: prevent infinite loop
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)

async def Move_Right_Attachment_Backward(speed, desired_position):
    motor.reset_relative_position(port.A,0)
    current_position = motor.relative_position(port.A)
    Distance_Degrees = int(desired_position)*10
    #print(Distance_Degrees)
    speed_to_move = -1*int(speed) *10
    while abs(motor.relative_position(port.A)) <= Distance_Degrees:
        motor.run(port.A,speed_to_move,acceleration=10000)
    motor.stop(port.A,stop=motor.BRAKE)
    #print(abs(motor.relative_position(port.A)))
    #await runloop.sleep_ms(1000) # wait for a second


async def Move_Right_Attachment_Forward(speed, desired_position):
    motor.reset_relative_position(port.A,0)
    current_position = motor.relative_position(port.A)
    Distance_Degrees = int(desired_position)*10
    #print(Distance_Degrees)
    speed_to_move = int(speed) *10
    while abs(motor.relative_position(port.A)) <= Distance_Degrees:
        motor.run(port.A,speed_to_move,acceleration=10000)
    motor.stop(port.A,stop=motor.BRAKE)
    #print(abs(motor.relative_position(port.A)))
    await runloop.sleep_ms(1000) # wait for a second


async def Move_Left_Attachment_Backward(speed, desired_position):
    motor.reset_relative_position(port.B,0)
    current_position = motor.relative_position(port.B)
    Distance_Degrees = int(desired_position)*10
    #print(Distance_Degrees)
    speed_to_move = -1*int(speed) *10
    while abs(motor.relative_position(port.B)) <= Distance_Degrees:
        motor.run(port.B,speed_to_move,acceleration=9000)
    motor.stop(port.B,stop=motor.BRAKE)
    #print(abs(motor.relative_position(port.B)))
    #await runloop.sleep_ms(1000) # wait for a second


async def Move_Left_Attachment_Forward(speed, desired_position):
    motor.reset_relative_position(port.B,0)
    current_position = motor.relative_position(port.B)
    Distance_Degrees = int(desired_position)*10
    #print(Distance_Degrees)
    speed_to_move = int(speed) * 10
    while abs(motor.relative_position(port.B)) <= Distance_Degrees:
        motor.run(port.B,speed_to_move,acceleration=10000)
    motor.stop(port.B,stop=motor.BRAKE)
    #print(abs(motor.relative_position(port.B)))
    await runloop.sleep_ms(1000) # wait for a second

async def runMission():
    yaw_value = 0
    speed_percent = 60
    await Gyro_Move_Forward_By_Distance(yaw_value,28.6,speed_percent) # Move Straight until the specified distance point and stop.
    await Gyro_Left_turn(29) # Turn right degrees given
    await Gyro_Right_turn(5) # Turn right degrees given
    await Gyro_Move_Backward_By_Distance(yaw_value,3.3,speed_percent) # Move Straight until the specified distance point and stop.
    await Gyro_Right_turn(55) # Turn right degrees given
    await Gyro_Move_Forward_By_Distance(yaw_value,1.2,speed_percent) # Move Straight until the specified distance point and stop.
    await Gyro_Left_turn(50) # Turn right degrees given
    await Gyro_Move_Backward_By_Distance(yaw_value,0.5,speed_percent) # Move Straight until the specified distance point and stop.
    await Gyro_Right_turn(98) # Turn right degrees given
    await Gyro_Move_Forward_By_Distance(yaw_value,0.5,speed_percent) # Move Straight until the specified distance point and stop.
    # speed_percent = 40
    await Gyro_Right_turn(5) # Turn right degrees given
    await Gyro_Move_Forward_By_Distance(yaw_value,1,speed_percent) # Move Straight until the specified distance point and stop.
    await Move_Left_Attachment_Forward(40, 45)
    #await Gyro_Left_turn(5) # Turn right degrees given
    await Gyro_Move_Forward_By_Distance(yaw_value,3,speed_percent) # Move Straight until the specified distance point and stop.
    await Move_Left_Attachment_Backward(40, 45)

    await Gyro_Move_Backward_By_Distance(yaw_value,1.3,speed_percent) # Move Straight until the specified distance point and stop.
    speed_percent = 80
    await Move_Right_Attachment_Forward(100, 40)
    await Move_Right_Attachment_Backward(20, 40)
    await Move_Right_Attachment_Forward(100, 40)
    await Move_Right_Attachment_Backward(20, 40)
    await Move_Right_Attachment_Forward(100, 40)
    await Move_Right_Attachment_Backward(20, 40)
    await Move_Right_Attachment_Forward(100, 40)
    await Move_Right_Attachment_Backward(20, 40)
    await Move_Right_Attachment_Forward(100, 40)
    await Move_Right_Attachment_Backward(20, 40)
runloop.run(runMission())