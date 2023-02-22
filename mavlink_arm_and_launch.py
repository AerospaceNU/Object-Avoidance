
# pymavlink to talk to sim
from pymavlink import mavutil

# import opencv to read range finder
import cv2

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

print("waiting for heartbeat...")

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
the_connection.mav.system_time_send(33, 30)


# Once connected, use 'the_connection' to get and send messages

# Set mode
print("Setting mode... ")
the_connection.mav.command_long_send(
the_connection.target_system, the_connection.target_component,
mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
1, 4, 0, 0, 0, 0, 0)
print("Mode set!")

#Arm drone
print("Arming... ")
the_connection.mav.command_long_send(
the_connection.target_system, the_connection.target_component,
mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0, 1, 0, 0, 0, 0, 0, 0)
print("Armed!")

# Takeoff
print("Taking off...")
the_connection.mav.command_long_send(
the_connection.target_system, the_connection.target_component,
mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0, 0, 0, 0, 0, 0, 0, 10)
print("Takeoff successful!")

# get camera feed
cap = cv2.VideoCapture('rtsp://127.0.0.1:5599')

# display
while(True):
    ret, frame = cap.read()
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()