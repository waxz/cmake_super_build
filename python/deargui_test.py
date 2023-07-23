import time
import math
import random
import struct
from paho.mqtt import client as mqtt_client
import base64
import re

import dearpygui.dearpygui as dpg

dpg.create_context()


def callback():
    print(dpg.get_value("unique_tag"))


QUIT = False


def quit_callback(sender):
    print("click quit")

    global QUIT
    QUIT = True
    print("QUIT", QUIT)


line_data_x = []
line_data_y1 = []
line_data_y2 = []
line_data_y3 = []
line_data_y4 = []
line_data_y5 = []
line_data_y6 = []
line_data_y7 = []
line_data_y8 = []
line_data_y9 = []
line_data_y10 = []
line_data_y11 = []
line_data_y12 = []
line_data_y13 = []


msg_count = 0

def clear_callback(sender):
    line_data_x.clear()
    line_data_y1.clear()
    line_data_y2.clear()
    line_data_y3.clear()
    line_data_y4.clear()
    line_data_y5.clear()
    line_data_y6.clear()
    line_data_y7.clear()
    line_data_y8.clear()
    line_data_y9.clear()
    line_data_y10.clear()
    line_data_y11.clear()
    line_data_y12.clear()
    line_data_y13.clear()
    global  msg_count
    msg_count = 0

update_x_axis = False
def update_x_axis_callback(sender):
    global  update_x_axis
    update_x_axis = not update_x_axis


def print_me(sender):
    print(f"Menu Item: {sender}")


with dpg.viewport_menu_bar():
    with dpg.menu(label="File"):
        dpg.add_menu_item(label="Save", callback=print_me)
        dpg.add_menu_item(label="Save As", callback=print_me)

        with dpg.menu(label="Settings"):
            dpg.add_menu_item(label="Setting 1", callback=print_me, check=True)
            dpg.add_menu_item(label="Setting 2", callback=print_me)

    dpg.add_menu_item(label="Help", callback=print_me)

    with dpg.menu(label="Widget Items"):
        dpg.add_checkbox(label="Pick Me", callback=print_me)
        dpg.add_button(label="Press Me", callback=print_me)
        dpg.add_color_picker(label="Color Me", callback=print_me)
    dpg.add_menu_item(label="Quit", callback=quit_callback)
    dpg.add_menu_item(label="CLear", callback=clear_callback)
    dpg.add_menu_item(label="UpdateXAxis", callback=update_x_axis_callback)


with dpg.window(label="Example callback"):
    dpg.add_button(label="Press me (print to output)", callback=callback)
    dpg.add_input_int(default_value=5, label="Input", tag="unique_tag")

with dpg.window(tag="Primary Window", pos=(0, 0)):
    dpg.add_text("Hello, world")
    dpg.add_button(label="Save")
    dpg.add_button(label="QUIT", callback=quit_callback)

    dpg.add_input_text(label="string", default_value="Quick brown fox")
    dpg.add_slider_float(label="float", default_value=0.273, max_value=1)

with dpg.window(label="Tutorial"):
    b0 = dpg.add_button(label="button 0")
    b1 = dpg.add_button(tag=100, label="Button 1")
    dpg.add_button(tag="Btn2", label="Button 2")

line_id = 0
text_id = dpg.generate_uuid()
with dpg.window(label="Tutorial Plot", pos=(500, 0)):
    with dpg.drawlist(width=300, height=300):  # or you could use dpg.add_drawlist and set parents manually

        line_id = dpg.draw_line((10, 10), (100, 100), color=(255, 0, 0, 255), thickness=1)
        text_id = dpg.draw_text((0, 0), "Origin", color=(250, 250, 250, 255), size=15, tag=text_id)
        dpg.draw_arrow((50, 70), (100, 65), color=(0, 200, 255), thickness=1, size=10, tag="arrow_1")
        #print("text_id:", text_id)
        #print("line_id:", line_id)

with dpg.window(label="Simple Plot", pos=(0, 500)):
    with dpg.plot(label="Bar Series", height=400, width=800):
        dpg.add_plot_legend()

        # create x axis
        dpg.add_plot_axis(dpg.mvXAxis, label="Student", no_gridlines=True)
        dpg.set_axis_limits(dpg.last_item(), 9, 33)
        dpg.set_axis_ticks(dpg.last_item(), (("S1", 11), ("S2", 21), ("S3", 31)))

        # create y axis
        with dpg.plot_axis(dpg.mvYAxis, label="Score"):
            dpg.set_axis_limits(dpg.last_item(), 0, 1000)
            dpg.add_bar_series([10, 20, 30], [100, 75, 90], label="Final Exam", weight=1, tag="bar_data_1")
            dpg.add_bar_series([11, 21, 31], [83, 75, 72], label="Midterm Exam", weight=1)
            dpg.add_bar_series([12, 22, 32], [42, 68, 23], label="Course Grade", weight=1)
with dpg.window(label="simple window", pos=(500, 500)):
    dpg.add_simple_plot(label="Simple Plot", min_scale=-1.0, max_scale=1.0, height=300, tag="plot 1")

command_input = []


with dpg.window(label="line window", pos=(0, 0)):

    with dpg.group(horizontal=True):

        with dpg.group(horizontal=False, width=800):
            dpg.add_input_text(label="stop", tag="command_text_1", default_value="move_command(0.0,0.0,0.0,0.0);")
            dpg.add_input_text(label="turn", tag="command_text_2", default_value="move_command(0.0,0.5,0.0,0.5);")
            dpg.add_input_text(label="run", tag="command_text_3", default_value="move_command(0.2,0.0,0.2,0.0);")
            dpg.add_input_text(label="update_rot", tag="command_text_4", default_value="update_rot(-2.204464040e-06, -0.07178068161010742, -2.204464040e-06, -0.07609415054321289);")
            dpg.add_input_text(label="update_forward", tag="command_text_5", default_value="update_forward(0.00041617400711402297, 0.00041617400711402297);")

        with dpg.group(horizontal=False, width=60):
            dpg.add_button(label="send",   callback= lambda  sender, app_data, user_data: dpg.get_value("command_text_1" ) and (command_input.append(dpg.get_value("command_text_1" ))))
            dpg.add_button(label="send",   callback= lambda  sender, app_data, user_data: dpg.get_value("command_text_2" ) and (command_input.append(dpg.get_value("command_text_2" ))))
            dpg.add_button(label="send",   callback= lambda  sender, app_data, user_data: dpg.get_value("command_text_3" ) and (command_input.append(dpg.get_value("command_text_3" ))))
            dpg.add_button(label="send",   callback= lambda  sender, app_data, user_data: dpg.get_value("command_text_4" ) and (command_input.append(dpg.get_value("command_text_4" ))))
            dpg.add_button(label="send",   callback= lambda  sender, app_data, user_data: dpg.get_value("command_text_5" ) and (command_input.append(dpg.get_value("command_text_5" ))))

    # dpg.add_int_value( label="msg count",tag="msg counter")
    dpg.add_input_int(label="msg counter", payload_type="ints", width=100, step=0, tag="msg counter")



    with dpg.plot(label="line plot", height=600, width=1800, query=True, no_menus=False, crosshairs=True):
        dpg.add_plot_legend()

        xaxis = dpg.add_plot_axis(dpg.mvXAxis, label="x", tag = "x_axis")
        # dpg.set_axis_limits(xaxis,0.0,10.0)

        y1axis = dpg.add_plot_axis(dpg.mvYAxis, label="y1")
        # dpg.set_axis_limits(y1axis,-1.5,1.5)

        y2axis = dpg.add_plot_axis(dpg.mvYAxis, label="y2")
        # dpg.set_axis_limits(y2axis,-1.5,1.5)

        y3axis = dpg.add_plot_axis(dpg.mvYAxis, label="y3")

        y4axis = dpg.add_plot_axis(dpg.mvYAxis, label="y4")

        # dpg.set_axis_limits(y2axis,-1.5,1.5)
        # dpg.set_axis_limits("x_axis", 0, 110)

        # dpg.add_tooltip(y1axis)

        y_max = 30
        dpg.add_line_series([0,y_max], [-1.5,1.5], tag='line_1', label="forward_cmd_1", parent=y1axis)
        dpg.add_line_series([0,y_max], [-1.5,1.5], tag='line_2', label="forward_fdb_1", parent=y1axis)
        dpg.add_line_series([0,y_max], [-1.5,1.5], tag='line_3', label="rotate_cmd_1", parent=y2axis)
        dpg.add_line_series([0,y_max], [-1.5,1.5], tag='line_4', label="rotate_fdb_1", parent=y2axis)
        dpg.add_line_series([0,y_max], [-1.5,1.5], tag='line_5', label="constrain_1", parent=y3axis)
        dpg.add_line_series([0,y_max], [-1.5,1.5], tag='line_6', label="forward_cmd_2", parent=y1axis)
        dpg.add_line_series([0,y_max], [-1.5,1.5], tag='line_7', label="forward_fdb_2", parent=y1axis)
        dpg.add_line_series([0,y_max], [-1.5,1.5], tag='line_8', label="rotate_cmd_2", parent=y2axis)
        dpg.add_line_series([0,y_max], [-1.5,1.5], tag='line_9', label="rotate_fdb_2", parent=y2axis)
        dpg.add_line_series([0,y_max], [-1.5,1.5], tag='line_10', label="constrain_2", parent=y3axis)

        dpg.add_line_series([0,y_max], [-1.5,1.5], tag='line_11', label="cmd_vel_linear_x", parent=y3axis)
        dpg.add_line_series([0,y_max], [-1.5,1.5], tag='line_12', label="cmd_vel_linear_y", parent=y3axis)
        dpg.add_line_series([0,y_max], [-1.5,1.5], tag='line_13', label="cmd_vel_angular_z", parent=y3axis)

        # dpg.add_plot_annotation(label="Center", default_value=(0.5, 0.5), color=[255, 255, 0, 255])


dpg.create_viewport(title='hello_robot', width=1800, height=800)
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.set_primary_window("Primary Window", True)

# dpg.start_dearpygui()
# The manual render loop must be created after setup_dearpygui
xx = 0
yy = 0
x_rot = 10
y_rot = 45
z_rot = 0

view = dpg.create_fps_matrix([0, 0, 50], 0.0, 0.0)
proj = dpg.create_perspective_matrix(math.pi * 45.0 / 180.0, 1.0, 0.1, 100)
model = dpg.create_rotation_matrix(math.pi * x_rot / 180.0, [1, 0, 0]) * \
        dpg.create_rotation_matrix(math.pi * y_rot / 180.0, [0, 1, 0]) * \
        dpg.create_rotation_matrix(math.pi * z_rot / 180.0, [0, 0, 1])

f1 = 0.0

sindatax = []
sindatay = []
for i in range(0, 500):
    sindatax.append(i / 1000)
    sindatay.append(0.5 + 0.5 * math.sin(50 * i / 1000))
ii = 500


#### MQTT


def decode_base64(data, altchars=b'+/'):
    """Decode base64, padding being optional.

    :param data: Base64 data as an ASCII byte string
    :returns: The decoded byte string.

    """
    data = re.sub(rb'[^a-zA-Z0-9%s]+' % altchars, b'', data)  # normalize
    missing_padding = len(data) % 4
    if missing_padding:
        data += b'=' * (4 - missing_padding)
    return base64.b64decode(data, altchars)


broker = 'broker.emqx.io'
broker = '127.0.0.1'

port = 1883
# topic = "/python/mqtt"
topic = "hello_robot"
topics = ["hello_robot", "hello_lua"]
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 1000)}'


def subscribe(client: mqtt_client):
    def on_message(_client, userdata, msg):
        # print(f"Received from `{msg.topic}` topic")

        if msg.topic == "hello_robot":

        # print(f"Received  {msg.payload}  ")

            barray = decode_base64(msg.payload)
            # print("barray:\n", barray)
            integers = struct.unpack('{}h'.format(len(barray) // 2), barray)
            float_array = [i * 0.0001 for i in integers]
            # global_float_array.extend(float_array)
            # print("integers:\n", integers)
            # print("float_array:\n", float_array)

            STATUS_NUM = 14
            float_array_0 = float_array[0::STATUS_NUM]
            float_array_1 = float_array[1::STATUS_NUM]
            float_array_2 = float_array[2::STATUS_NUM]
            float_array_3 = float_array[3::STATUS_NUM]
            float_array_4 = float_array[4::STATUS_NUM]
            float_array_5 = float_array[5::STATUS_NUM]
            float_array_6 = float_array[6::STATUS_NUM]
            float_array_7 = float_array[7::STATUS_NUM]
            float_array_8 = float_array[8::STATUS_NUM]
            float_array_9 = float_array[9::STATUS_NUM]
            float_array_10 = float_array[10::STATUS_NUM]
            
            float_array_11 = float_array[11::STATUS_NUM]
            float_array_12 = float_array[12::STATUS_NUM]
            float_array_13 = float_array[13::STATUS_NUM]
            
            line_data_y1.extend(float_array_1)
            line_data_y2.extend(float_array_2)
            line_data_y3.extend(float_array_3)
            line_data_y4.extend(float_array_4)
            line_data_y5.extend(float_array_5)
            line_data_y6.extend(float_array_6)
            line_data_y7.extend(float_array_7)
            line_data_y8.extend(float_array_8)
            line_data_y9.extend(float_array_9)
            line_data_y10.extend(float_array_10)
            line_data_y11.extend(float_array_11)
            line_data_y12.extend(float_array_12)
            line_data_y13.extend(float_array_13)

            if (len(line_data_x) == 0):
                sx = 0.0
            else:
                sx = line_data_x[-1]

            float_array_x = [tt + sx for tt in float_array_0]
            line_data_x.extend(float_array_x)
            # dpg.set_axis_limits("x",line_data_x[-1]-30.0, line_data_x[-1])
            # dpg.set_x_scroll("x_axis", line_data_x[-1])
            global  update_x_axis
            if update_x_axis:
                dpg.set_axis_limits("x_axis", line_data_x[-1] - 30 , line_data_x[-1]+2)
            else:
                dpg.set_axis_limits_auto("x_axis")

            dpg.configure_item('line_1', x=line_data_x, y=line_data_y1)
            dpg.configure_item('line_2', x=line_data_x, y=line_data_y2)

            dpg.configure_item('line_3', x=line_data_x, y=line_data_y3)
            dpg.configure_item('line_4', x=line_data_x, y=line_data_y4)
            dpg.configure_item('line_5', x=line_data_x, y=line_data_y5)
            dpg.configure_item('line_6', x=line_data_x, y=line_data_y6)
            dpg.configure_item('line_7', x=line_data_x, y=line_data_y7)
            dpg.configure_item('line_8', x=line_data_x, y=line_data_y8)
            dpg.configure_item('line_9', x=line_data_x, y=line_data_y9)
            dpg.configure_item('line_10', x=line_data_x, y=line_data_y10)


            dpg.configure_item('line_11', x=line_data_x, y=line_data_y11)
            dpg.configure_item('line_12', x=line_data_x, y=line_data_y12)
            dpg.configure_item('line_13', x=line_data_x, y=line_data_y13)

            # print(f"decode_base64 = ${decode_base64(msg.payload)}")


            global msg_count
            msg_count = msg_count + 1
            dpg.configure_item("msg counter", default_value = msg_count )
        elif msg.topic == "hello_lua":
            barray = decode_base64(msg.payload)
            print("get lua:\n", barray)

        return

    for t in topics:
        client.subscribe(t)
    client.on_message = on_message


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
            subscribe(client)
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)

    client.on_connect = on_connect
    # client.connect(broker, port)
    client.connect_async(broker, port)

    return client


def publish(client):
    msg_count = 0
    while True:
        time.sleep(0.01)
        msg = f"messages: {msg_count}"
        result = client.publish(topic, msg)
        # result: [0, 1]
        status = result[0]
        if status == 0:
            print(f"Send `{msg}` to topic `{topic}`")
        else:
            print(f"Failed to send message to topic {topic}")
        msg_count += 1

        # client.loop_misc()


#### MQTT

run_mqtt = True
mqtt_started = False
wait_mqtt = 0

while dpg.is_dearpygui_running() and not QUIT:
    time.sleep(0.1)

    # print("QUIT",QUIT)
    # insert here any code you would like to run in the render loop
    # you can manually stop by using stop_dearpygui()
    # print("this will run every frame")
    try:
        dpg.render_dearpygui_frame()
    except:
        pass


    if run_mqtt and not mqtt_started and (wait_mqtt > 5):
        client = connect_mqtt()
        client.loop_start()
        mqtt_started = True

    if run_mqtt and mqtt_started :
        client.loop_misc()

    wait_mqtt = wait_mqtt + 1
    # dpg.set_value("bar_data_1", ((0, 0), (xx, yy)))
    # data_x = [880,0]
    # data_y = [880,0]
    arrow_x, arrow_y = (xx, math.sqrt(10000 - xx * xx) + 10), (0, 0)
    dpg.configure_item('arrow_1', p1=arrow_x, p2=arrow_y)

    # dpg.apply_transform("arrow", proj*view*model)
    # dpg.apply_transform("arrow123", dpg.create_rotation_matrix(math.pi*45.0/180.0 , [0, 0, -1])*dpg.create_translation_matrix([150, 0]))
    # line_data_x.append(f1)
    # line_data_y1.append(math.sin(f1 + 1.0))
    # line_data_y2.append(math.sin(f1 + 2.0))
    # line_data_y3.append(math.cos(f1))
    # line_data_y4.append(math.cos(f1 + 1.0))
    # line_data_y5.append(math.cos(f1 + 2.0))
    # line_data_y6.append(0.5 * math.sin(f1))

    # dpg.configure_item('line_1', x=sindatax, y=sindatay)
    # dpg.configure_item('line_1', x=line_data_x, y=line_data_y1)
    # dpg.configure_item('line_2', x=line_data_x, y=line_data_y2)
    # dpg.configure_item('line_3', x=line_data_x, y=line_data_y3)
    # dpg.configure_item('line_4', x=line_data_x, y=line_data_y4)
    # dpg.configure_item('line_5', x=line_data_x, y=line_data_y5)
    # dpg.configure_item('line_6', x=line_data_x, y=line_data_y6)

    f1 = f1 + 0.01

    xx = xx + 1
    yy = yy + 1
    if (xx > 100):
        xx = 0

    if (yy > 500):
        yy = 0




    while len(command_input) > 0:
        data = command_input.pop()
        if(len(data) == 0 ):
            continue

        # data =  "hello(123);move_command(0.5,1.5,0.5,1.5);"

        # Standard Base64 Encoding
        encodedBytes = base64.b64encode(data.encode("utf-8"))
        encodedStr = str(encodedBytes, "utf-8")

        print(encodedStr)
        t = "hello_lua"
        result = client.publish(t, encodedStr)
        # result: [0, 1]
        status = result[0]
        if status == 0:
            print(f"Send `{encodedStr}` to topic `{t}`")
        else:
            print(f"Failed to send message to topic {t}")

dpg.destroy_context()

