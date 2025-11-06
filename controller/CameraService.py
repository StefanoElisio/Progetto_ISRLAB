import SimService
import numpy as np
import cv2
import copy
import math
import SharedData
import time

min_shape_area = 400
lower = np.array([0,50,0])
upper = np.array([150,255,255])
pixel = 256
memory = []
consistency_error = 5
sensor_range = 3.9
f_cam_number = None
r_cam_number = None

def camera_loop():
    global f_cam_number, r_cam_number
    _, sim = SimService.get_sim()
    shared = SharedData.get_shared()

    front_camera = sim.getObject(':/front_visionSensor')
    rear_camera = sim.getObject(':/rear_visionSensor')
    front_sensor = sim.getObject(':/front_proximitySensor')
    rear_sensor = sim.getObject(':/rear_proximitySensor')

    f_cam_number = front_camera
    r_cam_number = rear_camera

# sistemare: mettere che trova la posizione come prima cosa
    # togliere che si deve fermare per vedere la posizione e di conseguenza
    # poter aggiornare la posizione se il blocco viene spostato

    shared["setup"] += [True]
    print("[CameraService] avviato")
    while True:
        #print("viewing")
        global memory
        front_view = get_sensor_view(sim, front_camera)
        rear_view = get_sensor_view(sim, rear_camera)
        front_shape = shape_recognition(front_view)
        rear_shape = shape_recognition(rear_view)
        front_shape = color_recognition(front_view,front_shape)
        rear_shape = color_recognition(rear_view,rear_shape)
        new_f_shape = exclude_existing(front_shape, shared["blocks_pos"])
        new_r_shape = exclude_existing(rear_shape, shared["blocks_pos"])
        if len(new_f_shape) > 0 or len(new_r_shape) > 0:
            shared["state"] = "waiting"
            #time.sleep(0.2)
            front_shape = shape_recognition(front_view)
            rear_shape = shape_recognition(rear_view)
            front_shape = color_recognition(front_view, front_shape)
            rear_shape = color_recognition(rear_view, rear_shape)
            new_f_shape = exclude_existing(front_shape, shared["blocks_pos"])
            new_r_shape = exclude_existing(rear_shape, shared["blocks_pos"])
        else:
            continue
        if len(memory) > consistency_error:
            del memory[0]
        shapes = []
        if len(new_f_shape) > 0:
            f_shapes =  find_distance(sim, new_f_shape, front_sensor, front_camera)
            shapes += f_shapes
        if len(new_r_shape) > 0:
            r_shapes =  find_distance(sim, new_r_shape, rear_sensor, rear_camera)
            shapes += r_shapes
        exclude_existing(shapes, shared["blocks_pos"])
        shared["blocks_pos"] += shapes
        shared["state"] = None
        time.sleep(0.3)

def exclude_existing(shapes, mem):
    for sn in shapes:
        for so in mem:
            if sn[3] == so[3]:
                shapes.remove(sn)
    return shapes

def get_sensor_view(sim, camera):
    sim_image, resolution = sim.getVisionSensorImg(camera)
    unit8_image = sim.unpackUInt8Table(sim_image)
    unit8_red = []
    unit8_green = []
    unit8_blue = []
    for i in range(len(unit8_image)):
        if i % 3 == 0:
            unit8_blue.append(unit8_image[i])
        elif i % 3 == 1:
            unit8_green.append((unit8_image[i]))
        else:
            unit8_red.append((unit8_image[i]))
    unit8_rgb = [unit8_red,unit8_green,unit8_blue]
    channels_2d = [np.reshape(c, (pixel,pixel)) for c in unit8_rgb]
    np_image = np.array(channels_2d, dtype=np.uint8)
    image = np.flipud(np.stack(np_image, -1))
    return image

def shape_recognition(img):
    # Pre-processing
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    shape_recognized = []
    for contour in contours:

        area = cv2.contourArea(contour)
        if area < min_shape_area:
            continue

        perimeter = cv2.arcLength(contour, True)
        epsilon = 0.01 * perimeter
        approx = cv2.approxPolyDP(contour, epsilon, True)
        M = cv2.moments(contour)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        sides = len(approx)

        # Identifica la forma
        if sides <= 6:
            shape_name = "CUBO"
            shape_recognized.append((shape_name, cx, cy))

        #img = img.copy()
        #cv2.circle(img, (cx, cy), 7, (0, 0, 255), -1)

    #cv2.imshow('hsv',hsv)
    #cv2.imshow('maschera', mask)
    #cv2.imshow("Riconoscimento Forme", img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    return shape_recognized

def color_recognition(img,shape_list):
    for i in range(len(shape_list)):
        shape,x,y = shape_list[i]
        b = int(img[y, x, 0])
        g = int(img[y, x, 1])
        r = int(img[y, x, 2])
        if b > g and b > r:
            color = 'BLUE'
        elif g > b and g > r:
            color = 'GREEN'
        else:
            color = 'RED'
        shape_list[i] = (shape,x,y,color)
    return shape_list

def find_distance(sim, im_shapes, sensor, camera):
    shapes = []
    for shape in im_shapes:
        sim.setObjectOrientation(sensor, camera, [0, 0, 0])
        x = (shape[1] - pixel/2) / (pixel/2)
        y = (shape[2] - pixel/2) / (pixel/2)
        _,fov = sim.getObjectFloatParameter(camera, sim.visionfloatparam_perspective_angle)

        rx = math.atan(y * math.tan(fov / 2))
        ry = math.atan(x * math.tan(fov / 2))

        sim.setObjectOrientation(sensor, camera, [rx, -ry, 0])
        sim.step()

        res, dist, point, _, _ = sim.readProximitySensor(sensor)
        if res == 0:
            continue

        m = sim.getObjectMatrix(sensor, -1)
        z_x, z_y = m[2], m[6]
        s_x, s_y = m[3], m[7]

        print(z_x,z_y)
        block_x = s_x + dist * z_x
        block_y = s_y + dist * z_y
        shapes.append((shape[0], block_x, block_y, shape[3]))
        print(shapes)
    return shapes


