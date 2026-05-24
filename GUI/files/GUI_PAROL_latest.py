import tkinter
import tkinter.messagebox
import customtkinter
import matplotlib.pyplot as plt
import time
import roboticstoolbox as rp
import numpy as np
import platform
import os
from tkinter import filedialog
import PIL
from PIL import Image, ImageTk
import logging
import tkinter as tk
from tkinter import ttk
from tkinter.messagebox import showinfo
from tkinter import messagebox
import random
import multiprocessing
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
#from visual_kinematics.RobotSerial import *
import numpy as np
from math import pi
import PAROL6_ROBOT 
from datetime import datetime
import re
from Commander_feature_adapters import (
    build_vision_pick_sequence,
    clear_timestamp_table,
    export_timestamp_table as export_timestamp_table_file,
    load_config,
    modbus_manager,
    read_state,
    research_logger,
    save_config,
    update_state,
    vision_manager,
)

logging.basicConfig(level = logging.DEBUG,
    format='%(asctime)s.%(msecs)03d %(levelname)s:\t%(message)s',
    datefmt='%H:%M:%S'
)
#logging.disable(logging.DEBUG)


# Finds out where the program and images are stored
my_os = platform.system()
if my_os == "Windows":
    Image_path = os.path.join(os.path.dirname(os.path.realpath(__file__)))
    logging.debug("Os is Windows")
else:
    Image_path = os.path.join(os.path.dirname(os.path.realpath(__file__)))
    logging.debug("Os is Linux")
    
logging.debug(Image_path)

text_size = 14

# Globals
current_menu = "Jog"
Wrf_Trf = "TRF"
Current_Custom_pose_select = "Current"
Robot_sim = True
Real_robot = True
left_right_select = "Left"
Quick_grip = 0
Now_open_txt = ''
prev_string_shared = ""
Gripper_activate_deactivate = 1
Gripper_action_status = 1
Gripper_rel_dir = 1

# These are the values that are displayed in the gui and are updated every xx ms
x_value = ""
y_value = ""
z_value = ""
Rx_pos = ""
Ry_pos = ""
Rz_pos = ""

Joint1_value = ""
Joint2_value = ""
Joint3_value = ""
Joint4_value = ""
Joint5_value = ""
Joint6_value = ""

#customtkinter.set_appearance_mode("Light")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_appearance_mode("Dark")  # Industrial Precision HMI — dark variant
customtkinter.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"

left_jog_buttons = [0,0,0,0,0,0]
right_jog_buttons  =[0,0,0,0,0,0]
translation_buttons = [0,0,0,0,0,0]
rotation_buttons = [0,0,0,0,0,0]

left_frames_width = 430
right_frames_width = 300
bottom_frame_height = 96

# === Industrial Precision HMI palette — DARK variant (see AGENTS/DESIGN.md) ===
# Same design language as the light spec, remapped to a low-glare night palette
# for long shifts. Vibrant HMI blue, functional safety semantics, Polman gold
# branding — on slate-graphite surfaces with brushed-aluminum bezel borders.
UI_APP_BG          = "#0e1218"  # outer chrome / app background — deepest slate
UI_SURFACE         = "#1a202a"  # main panel face — graphite
UI_SURFACE_ALT     = "#222a36"  # recessed/header strips
UI_SURFACE_LOW     = "#171c25"  # subtle alt panels / LCD-readout bg
UI_SURFACE_HIGH    = "#2a3340"  # deeper recesses / data wells
UI_BORDER          = "#3a4254"  # silver bezel borders
UI_OUTLINE         = "#5c6378"  # stronger separators
UI_HANDLE          = "#3e4f63"  # drag handles
UI_ACCENT          = "#2e5bff"  # vibrant HMI blue
UI_ACCENT_DEEP     = "#4d75ff"  # pressed / hover (brighter on dark)
UI_ACCENT_SOFT     = "#1d2a4a"  # soft accent fill (dark tint)
UI_GOLD            = "#e5c363"  # Polman gold — brand highlights (lifted for dark)
UI_ON_SURFACE      = "#eaf2f9"  # primary text on dark surfaces
UI_ON_SURFACE_MUTE = "#a4adbf"  # inactive / hint text
UI_SUCCESS         = "#28c800"  # functional green — Start/Run
UI_DANGER          = "#e53935"  # functional red — Stop/Emergency
UI_WARN            = "#ffb800"  # functional amber — Reset/Standby

class CollapsibleFrame(customtkinter.CTkFrame):
    def __init__(self, parent, title, content_height=None, **kwargs):
        super().__init__(parent, corner_radius=8, border_width=1, border_color=UI_BORDER, fg_color=UI_SURFACE, **kwargs)
        self.columnconfigure(0, weight=1)
        
        self.header_frame = customtkinter.CTkFrame(self, fg_color="transparent")
        self.header_frame.grid(row=0, column=0, sticky="ew", padx=6, pady=4)
        self.header_frame.columnconfigure(0, weight=1)
        
        self.title_label = customtkinter.CTkLabel(self.header_frame, text=title, text_color=UI_ON_SURFACE, font=customtkinter.CTkFont(family="Inter", size=14, weight="bold"))
        self.title_label.grid(row=0, column=0, padx=6, pady=6, sticky="w")
        
        self.toggle_btn = customtkinter.CTkButton(
            self.header_frame,
            text="▼",
            width=22,
            height=22,
            corner_radius=11,
            fg_color=UI_ACCENT_SOFT,
            text_color=UI_ACCENT_DEEP,
            hover_color=UI_BORDER,
            font=customtkinter.CTkFont(size=10, weight="bold"),
            command=self.toggle
        )
        self.toggle_btn.grid(row=0, column=1, padx=6, pady=6, sticky="e")
        
        if content_height:
            self.content_frame = customtkinter.CTkScrollableFrame(self, fg_color="transparent", corner_radius=0, height=content_height)
        else:
            self.content_frame = customtkinter.CTkFrame(self, fg_color="transparent", corner_radius=0)
            
        self.content_frame.grid(row=1, column=0, sticky="ew", padx=6, pady=(0, 8))
        self.content_frame.columnconfigure((0, 1, 2, 3), weight=1)
        
        self.is_collapsed = False
        self.header_frame.bind("<Button-1>", lambda event: self.toggle())
        self.title_label.bind("<Button-1>", lambda event: self.toggle())

    def toggle(self):
        if self.is_collapsed:
            self.content_frame.grid(row=1, column=0, sticky="ew", padx=6, pady=(0, 8))
            self.toggle_btn.configure(text="▼")
            self.is_collapsed = False
        else:
            self.content_frame.grid_forget()
            self.toggle_btn.configure(text="▶")
            self.is_collapsed = True

prev_positions = np.array([0,0,0,0,0,0])
robot_pose = [0,0,0,0,0,0] #np.array([0,0,0,0,0,0])

padx_top_bot = 20
def GUI(shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons):
    


    app = customtkinter.CTk()
    customtkinter.set_widget_scaling(0.85)
    app.current_menu = "Jog"
    shared_string.value = b'PAROL6 commander v1.0'
    
    def _update_tab_highlights(active_name):
        # Map each tab button to the set of menu names that should keep it lit.
        # The Move button covers both "Jog" and "Cart" workspaces, so it must be
        # treated as active for either. Using a (button -> {names}) layout
        # guarantees each unique button is styled exactly once — otherwise a
        # later iteration could overwrite an earlier highlight.
        tabs = [
            (getattr(app, "move_mode_select_button", None), {"Jog", "Cart"}),
            (getattr(app, "I0_mode_select_button", None), {"I/O"}),
            (getattr(app, "Calibrate_button", None), {"Calibrate"}),
            (getattr(app, "Gripper_button", None), {"Gripper"}),
            (getattr(app, "Vision_button", None), {"Vision"}),
            (getattr(app, "Modbus_button", None), {"Modbus"}),
            (getattr(app, "Research_button", None), {"Research"}),
        ]
        for btn, active_for in tabs:
            if btn is None:
                continue
            if active_name in active_for:
                btn.configure(fg_color=UI_ACCENT, hover_color=UI_ACCENT_DEEP, text_color="#ffffff", font=customtkinter.CTkFont(family="Segoe UI Symbol", size=15, weight="bold"))
            else:
                btn.configure(fg_color="transparent", hover_color=UI_ACCENT_SOFT, text_color=UI_ON_SURFACE_MUTE, font=customtkinter.CTkFont(family="Segoe UI Symbol", size=15, weight="normal"))

    #logging.debug(left_jog_buttons)
    #logging.debug(Joint_jog_buttons)
    logging.debug("I RUN")
        # configure window
    app.title("Source controller.py")
    app.geometry("1920x1080")
    app.attributes('-topmost',False)
    # Add app icon  
    try:
        logo_png_path = os.path.join(Image_path, "logo_polman_icon.png")
        if os.path.exists(logo_png_path):
            icon_img = Image.open(logo_png_path)
            app.icon_photo = ImageTk.PhotoImage(icon_img)
            app.iconphoto(True, app.icon_photo)
        else:
            logo_krug_path = os.path.join(Image_path, "logo_krug_viri.png")
            if os.path.exists(logo_krug_path):
                Image.MAX_IMAGE_PIXELS = None
                icon_img = Image.open(logo_krug_path)
                icon_img = icon_img.resize((256, 256), Image.Resampling.LANCZOS)
                app.icon_photo = ImageTk.PhotoImage(icon_img)
                app.iconphoto(True, app.icon_photo)
            elif my_os == "Windows":
                logo = (os.path.join(Image_path, "logo.ico"))
                if os.path.exists(logo):
                    app.iconbitmap(logo)
    except Exception as e:
        logging.error(f"Error loading application icon: {e}")
        if my_os == "Windows":
            try:
                logo = (os.path.join(Image_path, "logo.ico"))
                app.iconbitmap(logo)
            except Exception:
                pass


    # configure grid layout (4x4) wight 0 znači da je fixed, 1 znači da scale radi?
    app.configure(fg_color=UI_APP_BG)
    app.layout_sizes = {"left": left_frames_width, "right": right_frames_width, "bottom": bottom_frame_height}
    app.layout_collapsed = {"left": False, "right": False, "bottom": False}
    app.resize_state = {}
    app.grid_columnconfigure(0, weight=0, minsize=app.layout_sizes["left"])
    app.grid_columnconfigure((1,2), weight=1)
    app.grid_columnconfigure(3, weight=0, minsize=app.layout_sizes["right"])
    app.grid_rowconfigure(0, weight=0)
    app.grid_rowconfigure(1, weight=3)
    app.grid_rowconfigure(2, weight=0)
    app.grid_rowconfigure(3, weight=2)
    app.grid_rowconfigure(4, weight=0, minsize=app.layout_sizes["bottom"]) 
    
    #images

    # dodaj plot koji je popup, dodaj help button, I/O je isto popup zasad i tamo je i gripper
    def top_frames():

        # frames for top panel mode selection section
        app.menu_select_frame = customtkinter.CTkFrame(app,height = 0,width=150, corner_radius=8, fg_color=UI_SURFACE, border_width=1, border_color=UI_BORDER)
        app.menu_select_frame.grid(row=0, column=0, columnspan=4, padx=(5,5), pady=5,sticky="new")
        app.menu_select_frame.grid_columnconfigure(0, weight=0)
        app.menu_select_frame.grid_rowconfigure(0, weight=0)

        # Tab glyph icons (Unicode, no asset files required).
        # Picked from the BMP "Misc Technical / Symbols" blocks so they render
        # in any default Tk font on Windows/Linux/macOS.
        _tab_font = customtkinter.CTkFont(family='Segoe UI Symbol', size=15)

        # Move button — compass / move arrows
        app.move_mode_select_button = customtkinter.CTkButton(app.menu_select_frame,text="✥  Move", font=_tab_font, command = raise_frame_jog)
        app.move_mode_select_button.grid(row=0, column=0, padx=(padx_top_bot,0),pady = 5,sticky="nw")

        # I/O button — bidirectional arrows
        app.I0_mode_select_button = customtkinter.CTkButton(app.menu_select_frame,text="⇅  I/O", font=_tab_font, command = raise_frame_IO)
        app.I0_mode_select_button.grid(row=0, column=1, padx=(padx_top_bot,0),pady = 5,sticky="nw")

        # Calibrate button — gear / settings
        app.Calibrate_button = customtkinter.CTkButton(app.menu_select_frame,text="⚙  Calibrate", font=_tab_font, command = raise_calibrate_frame)
        app.Calibrate_button.grid(row=0, column=2, padx=(padx_top_bot,0) ,pady = 5,sticky="nw")

        # Gripper button — pinch / clamp
        app.Gripper_button = customtkinter.CTkButton(app.menu_select_frame,text="⊓  Gripper", font=_tab_font, command = raise_gripper_frame)
        app.Gripper_button.grid(row=0, column=3, padx=(padx_top_bot,0) ,pady = 5,sticky="nw")

        # Vision button — camera lens / eye
        app.Vision_button = customtkinter.CTkButton(app.menu_select_frame,text="◉  Vision", font=_tab_font, command = raise_vision_frame)
        app.Vision_button.grid(row=0, column=4, padx=(padx_top_bot,0) ,pady = 5,sticky="nw")

        # Modbus button — bolt / live link
        app.Modbus_button = customtkinter.CTkButton(app.menu_select_frame,text="⚡  Modbus", font=_tab_font, command = raise_modbus_frame)
        app.Modbus_button.grid(row=0, column=5, padx=(padx_top_bot,0) ,pady = 5,sticky="nw")

        # Research button — chart / analytics
        app.Research_button = customtkinter.CTkButton(app.menu_select_frame,text="▤  Research", font=_tab_font, command = raise_research_frame)
        app.Research_button.grid(row=0, column=6, padx=(padx_top_bot,0) ,pady = 5,sticky="nw")

        app.fw_label = customtkinter.CTkLabel(app.menu_select_frame, text="Source controller fw v1.0.0", text_color=UI_GOLD, font=customtkinter.CTkFont(family='JetBrains Mono', size=12, weight='bold'))
        app.fw_label.grid(row=0, column=7, padx=(20,10), pady=5 ,sticky="ne")

        # help button
        help_image =Image.open(os.path.join(Image_path, "help.png"))
        app.help_button_image = customtkinter.CTkImage(help_image, size=(25, 25))
        
        app.help_button = customtkinter.CTkButton(app.menu_select_frame, corner_radius=0, height=1, border_spacing=10,
                                                fg_color="transparent", text_color=("gray10", "gray90"),
                                                image=app.help_button_image, anchor="CENTER",text = "",hover = 0,command = Open_help) #hover = 0
        app.help_button.grid(row=0, column=8, padx=(10,0), sticky="news")


    def bottom_frames():

        #frames for bottom panel section
        app.bottom_select_frame = customtkinter.CTkScrollableFrame(app, orientation="horizontal", height=bottom_frame_height, corner_radius=8, fg_color=UI_SURFACE, border_width=1, border_color=UI_BORDER)
        app.bottom_select_frame.grid(row=4, column=0, columnspan=4, padx=(5,5), pady=2, sticky="sew")
        app.bottom_select_frame.grid_columnconfigure(0, weight=0)

        # radio button left
        app.radio_button_sim = customtkinter.CTkRadioButton(master=app.bottom_select_frame, text="Simulator",  value=2,command = Select_simulator)
        app.radio_button_sim.grid(row=3, column=3, pady=10, padx=padx_top_bot, sticky="e")
        app.radio_button_sim.select()

        app.radio_button_real = customtkinter.CTkRadioButton(master=app.bottom_select_frame, text="Real robot",  value=2,command = Select_real_robot)
        app.radio_button_real.grid(row=3, column=4, pady=10, padx=padx_top_bot, sticky="e")
        app.radio_button_real.select()

        app.COMPORT = customtkinter.CTkEntry(app.bottom_select_frame, width= 150)
        app.COMPORT.grid(row=3, column=5, padx=(0, 0),pady=(3,3),sticky="E")

        # Add a helpful label for macOS users
        if my_os == "Darwin":
            app.COMPORT_label = customtkinter.CTkLabel(app.bottom_select_frame, text="Port (e.g., /dev/tty.usbmodem*)", font=customtkinter.CTkFont(size=10))
            app.COMPORT_label.grid(row=2, column=5, padx=(0, 0), pady=(3,0), sticky="E")
        elif my_os == "Windows":
            app.COMPORT_label = customtkinter.CTkLabel(app.bottom_select_frame, text="Port (e.g., 3 for COM3)", font=customtkinter.CTkFont(size=10))
            app.COMPORT_label.grid(row=2, column=5, padx=(0, 0), pady=(3,0), sticky="E")
        else:  # Linux
            app.COMPORT_label = customtkinter.CTkLabel(app.bottom_select_frame, text="Port (e.g., 0 for ttyACM0)", font=customtkinter.CTkFont(size=10))
            app.COMPORT_label.grid(row=2, column=5, padx=(0, 0), pady=(3,0), sticky="E")

        app.Connect_button = customtkinter.CTkButton(app.bottom_select_frame,text="Connect", font = customtkinter.CTkFont(family='Inter', size=15, weight='bold'), fg_color=UI_ACCENT, hover_color=UI_ACCENT_DEEP, text_color="#ffffff", command = Set_comm_port)
        app.Connect_button.grid(row=3, column=8, padx=padx_top_bot,pady = 10,sticky="e")

        app.Clear_error = customtkinter.CTkButton(app.bottom_select_frame,text="Clear error", font = customtkinter.CTkFont(family='Inter', size=15, weight='bold'), fg_color=UI_WARN, hover_color="#cc9300", text_color="#241a00", command = Clear_error)
        app.Clear_error.grid(row=3, column=9, padx=padx_top_bot,pady = 10,sticky="e")

        #app.enable_disable = customtkinter.CTkButton(app.bottom_select_frame,text="Enable", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'))
        #app.enable_disable.grid(row=3, column=9, padx=padx_top_bot,pady = 10,sticky="e")

        #app.home = customtkinter.CTkButton(app.bottom_select_frame,text="Home", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command = Home_robot)
        #app.home.grid(row=3, column=10, padx=padx_top_bot,pady = 10,sticky="e")

        app.estop_status = customtkinter.CTkLabel(app.bottom_select_frame, text="", font=customtkinter.CTkFont(size=15))
        app.estop_status.grid(row=3, column=11, padx=(0,0), pady=10 ,sticky="ne")


    app.left_scrollable_container = customtkinter.CTkScrollableFrame(app, width=left_frames_width, corner_radius=8, fg_color=UI_SURFACE, border_width=1, border_color=UI_BORDER)
    app.left_scrollable_container.grid(row=1, column=0, columnspan=1, rowspan=3, padx=(5,0), pady=5, sticky="news")
    app.left_scrollable_container.grid_columnconfigure(0, weight=1)

    app.left_upper_stack = customtkinter.CTkFrame(app.left_scrollable_container, fg_color="transparent", corner_radius=0)
    app.left_upper_stack.grid(row=0, column=0, sticky="ew", pady=(0, 5))
    app.left_upper_stack.grid_columnconfigure(0, weight=1)
    app.left_upper_stack.grid_rowconfigure(0, weight=1)

    app.jog_frame = CollapsibleFrame(app.left_upper_stack, title="Joint Jog Controls")
    app.jog_frame.grid(row=0, column=0, sticky="nsew")
    app.jog_frame.content_frame.grid_columnconfigure(0, weight=0)
    app.jog_frame.content_frame.grid_columnconfigure(1, weight=0)
    app.jog_frame.content_frame.grid_columnconfigure(2, weight=1)
    app.jog_frame.content_frame.grid_columnconfigure(3, weight=0)
    app.jog_frame.content_frame.grid_rowconfigure(0, weight=0)


    def joint_jog_frames():
        #jog frame
        app.joint_jog = customtkinter.CTkButton(app.jog_frame.content_frame,text="Joint jog", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command = raise_frame_jog)
        app.joint_jog.grid(row=0, column=0, padx=20,pady = (10,20),sticky="news")

        app.cart_jog = customtkinter.CTkButton(app.jog_frame.content_frame,text="Cartesian jog", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command = raise_frame_cart)
        app.cart_jog.grid(row=0, column=1, padx=20,pady = (10,20),sticky="news")

        joint_names = ['Base', 'Shoulder', 'Elbow', 'Wrist 1', 'Wrist 2', 'Wrist 3']

        def button_press_left(event=None, var = 0):
            left_jog_buttons[var] = 1
            Joint_jog_buttons[var] = 1
            logging.debug(left_jog_buttons)
            logging.debug("Joint jog press " + str(list(Joint_jog_buttons)))
            

        def button_rel_left(event=None, var = 0):
            left_jog_buttons[var] = 0
            Joint_jog_buttons[var] = 0
            logging.debug(left_jog_buttons)
            logging.debug("Joint jog release " + str(list(Joint_jog_buttons)))
            

        def button_press_right(event=None, var = 0):
            right_jog_buttons[var] = 1
            Joint_jog_buttons[var + 6] = 1
            logging.debug(right_jog_buttons)
            logging.debug("Joint jog press " + str(list(Joint_jog_buttons)))
            

        def button_rel_right(event=None, var = 0):
            right_jog_buttons[var] = 0
            Joint_jog_buttons[var + 6] = 0
            logging.debug(right_jog_buttons)
            logging.debug("Joint jog release " + str(list(Joint_jog_buttons)))
            

        true_image =Image.open(os.path.join(Image_path, "button_arrow_1.png"))
        rotated_image = true_image.rotate(90)
        rotated_image2 = true_image.rotate(270)

        app.move_arrow = {}
        app.move_arrow_right = {}
        app.progress_bar_joints = {}

        for y in range(0,6):

            def make_lambda1(x):
                return lambda ev:button_press_left(ev,x)

            def make_lambda2(x):
                return lambda ev:button_rel_left(ev,x)

            def make_lambda3(x):
                return lambda ev:button_press_right(ev,x)

            def make_lambda4(x):
                return lambda ev:button_rel_right(ev,x)


            # Labels of joints
            app.Base_label = customtkinter.CTkLabel(app.jog_frame.content_frame, text=joint_names[y], anchor="w",font=customtkinter.CTkFont(size=14))
            app.Base_label.grid(row=y+1, column=0, padx=(10, 0))

            #progress bars for joints 
            app.progress_bar_joints[y] = customtkinter.CTkProgressBar(app.jog_frame.content_frame, orientation="horizontal",height = 5)
            app.progress_bar_joints[y].grid(row=1 + y, column=2 , rowspan=1, padx=(10, 10), pady=(8, 8), sticky="ew")
        

            app.move_arrow_image = customtkinter.CTkImage(rotated_image, size=(38, 38))
            app.move_arrow[y] = customtkinter.CTkButton(app.jog_frame.content_frame, corner_radius=0, height=10, border_spacing=5,
                                                    fg_color="transparent", text_color=("gray10", "gray90"),
                                                    image=app.move_arrow_image, anchor="CENTER",text = "",hover = 0) #hover = 0
            app.move_arrow[y].grid(row=1 + y, column=1, sticky="news")
            app.move_arrow[y].bind('<ButtonPress-1>',make_lambda1(y))
            app.move_arrow[y].bind('<ButtonRelease-1>',make_lambda2(y))

            app.move_arrow_image_right = customtkinter.CTkImage(rotated_image2, size=(38, 38))
            app.move_arrow_right[y]  = customtkinter.CTkButton(app.jog_frame.content_frame, corner_radius=0, height=10, border_spacing=5,
                                                    fg_color="transparent", text_color=("gray10", "gray90"),
                                                    anchor="CENTER",text = "",image=app.move_arrow_image_right,hover = 0) #hover = 0
            app.move_arrow_right[y].grid(row=1 + y, column=3, sticky="news")
            app.move_arrow_right[y].bind('<ButtonPress-1>',make_lambda3(y))
            app.move_arrow_right[y].bind('<ButtonRelease-1>',make_lambda4(y))


    app.cart_frame = CollapsibleFrame(app.left_upper_stack, title="Cartesian Jog Controls")
    app.cart_frame.grid(row=0, column=0, sticky="nsew")
    app.cart_frame.content_frame.grid_rowconfigure(0, weight=0)


    app.IO_frame = CollapsibleFrame(app.left_upper_stack, title="I/O Signals Matrix")
    app.IO_frame.grid(row=0, column=0, sticky="nsew")
    app.IO_frame.content_frame.grid_columnconfigure(0, weight=0)
    app.IO_frame.content_frame.grid_rowconfigure(0, weight=0)

    app.Calibrate_frame = CollapsibleFrame(app.left_upper_stack, title="Calibration & Motor Controls")
    app.Calibrate_frame.grid(row=0, column=0, sticky="nsew")
    app.Calibrate_frame.content_frame.grid_columnconfigure(0, weight=0)
    app.Calibrate_frame.content_frame.grid_rowconfigure(0, weight=0)


    app.Gripper_frame = CollapsibleFrame(app.left_upper_stack, title="Gripper Parameters & Feedback")
    app.Gripper_frame.grid(row=0, column=0, sticky="nsew")
    app.Gripper_frame.content_frame.grid_columnconfigure(0, weight=0)
    app.Gripper_frame.content_frame.grid_rowconfigure(0, weight=0)

    app.vision_left_width = 850
    app.vision_frame = customtkinter.CTkFrame(app,height = 100, width = left_frames_width, corner_radius=8, fg_color=UI_SURFACE, border_width=1, border_color=UI_BORDER)
    app.vision_frame.grid(row=1, column=0, columnspan=4,rowspan=4,  padx=(5,5), pady=5, sticky="news")
    app.vision_frame.grid_columnconfigure(0, weight=0, minsize=app.vision_left_width)
    app.vision_frame.grid_columnconfigure(1, weight=0)
    app.vision_frame.grid_columnconfigure(2, weight=1)
    app.vision_frame.grid_rowconfigure(1, weight=1)

    app.modbus_left_width = 850
    app.modbus_top_height = 450
    app.modbus_frame = customtkinter.CTkFrame(app, corner_radius=8, fg_color=UI_SURFACE, border_width=1, border_color=UI_BORDER)
    app.modbus_frame.grid(row=1, column=0, columnspan=4,rowspan=4,  padx=(5,5), pady=5, sticky="news")
    app.modbus_frame.grid_columnconfigure(0, weight=0, minsize=app.modbus_left_width)
    app.modbus_frame.grid_columnconfigure(1, weight=0)
    app.modbus_frame.grid_columnconfigure(2, weight=1)
    app.modbus_frame.grid_rowconfigure(2, weight=0, minsize=app.modbus_top_height)
    app.modbus_frame.grid_rowconfigure(3, weight=0)
    app.modbus_frame.grid_rowconfigure(4, weight=1)

    app.research_left_width = 850
    app.research_top_height = 500
    app.research_frame = customtkinter.CTkFrame(app,height = 100, width = left_frames_width, corner_radius=8, fg_color=UI_SURFACE, border_width=1, border_color=UI_BORDER)
    app.research_frame.grid(row=1, column=0, columnspan=4,rowspan=4,  padx=(5,5), pady=5, sticky="news")
    app.research_frame.grid_columnconfigure(0, weight=0, minsize=app.research_left_width)
    app.research_frame.grid_columnconfigure(1, weight=0)
    app.research_frame.grid_columnconfigure(2, weight=1)
    app.research_frame.grid_rowconfigure(2, weight=0, minsize=app.research_top_height)
    app.research_frame.grid_rowconfigure(3, weight=0)
    app.research_frame.grid_rowconfigure(4, weight=1)

    def IO_frame():

        app.Input1 = customtkinter.CTkLabel(app.IO_frame.content_frame, text="INPUT 1: " + str(InOut_in[0]).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.Input1.grid(row=0, column=0, padx=20,pady = (10,20),sticky="news")

        app.Input2 = customtkinter.CTkLabel(app.IO_frame.content_frame, text="INPUT 2: " + str(InOut_in[1]).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.Input2.grid(row=1, column=0, padx=20,pady = (10,20),sticky="news")

        app.ESTOP_STATUS = customtkinter.CTkLabel(app.IO_frame.content_frame, text="ESTOP: " + str(InOut_in[4]).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.ESTOP_STATUS.grid(row=2, column=0, padx=20,pady = (10,20),sticky="news")

        app.OUTPUT_1_LABEL = customtkinter.CTkLabel(app.IO_frame.content_frame, text="OUTPUT 1 is: " + str(InOut_out[2]).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.OUTPUT_1_LABEL.grid(row=3, column=0, padx=20,pady = (10,20),sticky="news")

        app.Set_1_low = customtkinter.CTkButton(app.IO_frame.content_frame,text="LOW", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command= lambda: Set_output_1(0))
        app.Set_1_low.grid(row=3, column=1, padx=20,pady = (10,20),sticky="news")

        app.Set_1_high = customtkinter.CTkButton(app.IO_frame.content_frame,text="HIGH", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command= lambda: Set_output_1(1))
        app.Set_1_high.grid(row=3, column=2, padx=20,pady = (10,20),sticky="news")

        app.OUTPUT_2_LABEL = customtkinter.CTkLabel(app.IO_frame.content_frame, text="OUTPUT 2 is: " + str(InOut_out[3]).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.OUTPUT_2_LABEL.grid(row=4, column=0, padx=20,pady = (10,20),sticky="news")

        app.Set_2_low = customtkinter.CTkButton(app.IO_frame.content_frame,text="LOW", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command= lambda: Set_output_2(0))
        app.Set_2_low.grid(row=4, column=1, padx=20,pady = (10,20),sticky="news")

        app.Set_2_high = customtkinter.CTkButton(app.IO_frame.content_frame,text="HIGH", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command= lambda: Set_output_2(1))
        app.Set_2_high.grid(row=4, column=2, padx=20,pady = (10,20),sticky="news")


    def Calibrate_frame():

        app.disable_motor = customtkinter.CTkButton(app.Calibrate_frame.content_frame,text="Disable motor", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command = demo_stop)
        app.disable_motor.grid(row=2, column=1, padx=20,pady = (10,20),sticky="news")

        app.enable_motor = customtkinter.CTkButton(app.Calibrate_frame.content_frame,text="Enable motor", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command = demo_stop)
        app.enable_motor.grid(row=2, column=2, padx=20,pady = (10,20),sticky="news")

        app.Go_2_limit = customtkinter.CTkButton(app.Calibrate_frame.content_frame,text="Go to limit", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command = demo_stop)
        app.Go_2_limit.grid(row=2, column=3, padx=20,pady = (10,20),sticky="news")

        app.joint_select = customtkinter.CTkOptionMenu(app.Calibrate_frame.content_frame, values=["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5","Joint 6"])
        app.joint_select.grid(row=2, column=4,padx=(5, 0) )


    def Gripper_frame():
        # Devicee info
        # Activate/deactivate
        # Auto release direction
        # Calibrate

        app.Gripper_ID = customtkinter.CTkLabel(app.Gripper_frame.content_frame, text="Gripper ID is: " + str(0), font=customtkinter.CTkFont(size=text_size))
        app.Gripper_ID.grid(row=0, column=0, padx=20,pady = (10,20),sticky="news")

        app.grip_cal_status = customtkinter.CTkLabel(app.Gripper_frame.content_frame, text="Calibration status is: " + str(0), font=customtkinter.CTkFont(size=text_size))
        app.grip_cal_status.grid(row=0, column=1, padx=20,pady = (10,20),sticky="news")

        app.Error_status_grip = customtkinter.CTkLabel(app.Gripper_frame.content_frame, text="Error status is: " + str(InOut_out[2]).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.Error_status_grip.grid(row=0, column=2, padx=20,pady = (10,20),sticky="news")

        app.grip_activate_radio = customtkinter.CTkRadioButton(master=app.Gripper_frame.content_frame, text="Activate",  value=2,command = Select_gripper_activate)
        app.grip_activate_radio.grid(row=1, column=0, pady=10, padx=padx_top_bot, sticky="news")
        app.grip_activate_radio.select()

        app.grip_calibrate = customtkinter.CTkButton(app.Gripper_frame.content_frame,text="Calibrate gripper", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command = Gripper_calibrate)
        app.grip_calibrate.grid(row=1, column=1, padx=20,pady = (10,20),sticky="news")

        app.grip_clear_error = customtkinter.CTkButton(app.Gripper_frame.content_frame,text="Clear gripper error", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command = Gripper_clear_error)
        app.grip_clear_error.grid(row=1, column=2, padx=20,pady = (10,20),sticky="news")

        app.grip_setpoints = customtkinter.CTkLabel(app.Gripper_frame.content_frame, text="Command parameters", font=customtkinter.CTkFont(size=text_size))
        app.grip_setpoints.grid(row=2, column=0, padx=20,pady = (10,20),sticky="news")

        # Pos
        #app.grip_pos_label = customtkinter.CTkLabel(app.Gripper_frame.content_frame, text="Position setpoint", font=customtkinter.CTkFont(size=text_size))
        #pp.grip_pos_label.grid(row=3, column=0, padx=20,pady = (5,5),sticky="news")

        app.grip_pos_slider = customtkinter.CTkSlider(app.Gripper_frame.content_frame,from_ = 0, to = 255,number_of_steps=255)
        app.grip_pos_slider.set(10)
        app.grip_pos_slider.grid(row=3, column=1,columnspan=1, padx=(0, 10), pady=(5, 5), sticky="news")

        app.grip_pos_percent = customtkinter.CTkLabel(app.Gripper_frame.content_frame,text="100%", font = customtkinter.CTkFont(size=18, family='TkDefaultFont'))
        app.grip_pos_percent.grid(row=3, column=2, padx=5,pady = (5,5),sticky="news")

        app.grip_pos_entry = customtkinter.CTkEntry(app.Gripper_frame.content_frame, width= 150)
        app.grip_pos_entry.grid(row= 3, column=3, padx=(0, 0),pady=(3,3),sticky="E")

        app.grip_pos_set = customtkinter.CTkButton(app.Gripper_frame.content_frame,text="Position setpoint", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command = Set_gripper_pos)
        app.grip_pos_set.grid(row=3, column=0, padx=20,pady = (3,3),sticky="news")


        # Speed
        #app.grip_speed_label = customtkinter.CTkLabel(app.Gripper_frame.content_frame, text="Speed setpoint", font=customtkinter.CTkFont(size=text_size))
        #app.grip_speed_label.grid(row=5, column=0, padx=20,pady = (5,5),sticky="news")

        app.grip_speed_slider = customtkinter.CTkSlider(app.Gripper_frame.content_frame,from_ = 0, to = 255,number_of_steps=255)
        app.grip_speed_slider.set(50)
        app.grip_speed_slider.grid(row=5, column=1,columnspan=1, padx=(0, 10), pady=(5, 5), sticky="news")

        app.grip_speed_percent = customtkinter.CTkLabel(app.Gripper_frame.content_frame,text="100%", font = customtkinter.CTkFont(size=18, family='TkDefaultFont'))
        app.grip_speed_percent.grid(row=5, column=2, padx=5,pady = (5,5),sticky="news")

        app.grip_speed_entry = customtkinter.CTkEntry(app.Gripper_frame.content_frame, width= 150)
        app.grip_speed_entry.grid(row= 5, column=3, padx=(0, 0),pady=(3,3),sticky="E")

        app.grip_speed_set = customtkinter.CTkButton(app.Gripper_frame.content_frame,text="Speed setpoint", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command = Set_gripper_vel)
        app.grip_speed_set.grid(row=5, column=0, padx=20,pady = (3,3),sticky="news")  



         # Current
        #app.grip_current_label = customtkinter.CTkLabel(app.Gripper_frame.content_frame, text="Current setpoint", font=customtkinter.CTkFont(size=text_size))
        #app.grip_current_label.grid(row=7, column=0, padx=20,pady = (5,5),sticky="news")

        app.grip_current_slider = customtkinter.CTkSlider(app.Gripper_frame.content_frame,from_ = 100, to = 1000,number_of_steps=900)
        app.grip_current_slider.set(180)
        app.grip_current_slider.grid(row=7, column=1,columnspan=1, padx=(0, 10), pady=(5, 5), sticky="news")

        app.grip_current_percent = customtkinter.CTkLabel(app.Gripper_frame.content_frame,text="100%", font = customtkinter.CTkFont(size=18, family='TkDefaultFont'))
        app.grip_current_percent.grid(row=7, column=2, padx=5,pady = (5,5),sticky="news")

        app.grip_current_entry = customtkinter.CTkEntry(app.Gripper_frame.content_frame, width= 150)
        app.grip_current_entry.grid(row= 7, column=3, padx=(0, 0),pady=(3,3),sticky="E")

        app.grip_current_set = customtkinter.CTkButton(app.Gripper_frame.content_frame,text="Current setpoint", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command = Set_gripper_cur)
        app.grip_current_set.grid(row=7, column=0, padx=20,pady = (3,3),sticky="news")  

        app.grip_set = customtkinter.CTkButton(app.Gripper_frame.content_frame,text="Move GoTo", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command = Gripper_set_values)
        app.grip_set.grid(row=8, column=0, padx=20,pady = (3,3),sticky="news")  
        app.change_ID = customtkinter.CTkButton(app.Gripper_frame.content_frame,text="Change gripper ID", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command = Change_gripper_ID)
        app.change_ID.grid(row=8, column=1, padx=20,pady = (3,3),sticky="news") 
        app.grip_ID_entry = customtkinter.CTkEntry(app.Gripper_frame.content_frame, width= 150)
        app.grip_ID_entry.grid(row= 8, column=2, padx=(0, 0),pady=(3,3),sticky="E")


        # Feedback

        app.grip_empty = customtkinter.CTkLabel(app.Gripper_frame.content_frame,text="", font = customtkinter.CTkFont(size=18, family='TkDefaultFont'))
        app.grip_empty.grid(row=9, column=0, padx=20,pady = (5,5),sticky="news")

        app.grip_feedback = customtkinter.CTkLabel(app.Gripper_frame.content_frame,text="Gripper feedback", font = customtkinter.CTkFont(size=18, family='TkDefaultFont'))
        app.grip_feedback.grid(row=10, column=0, padx=20,pady = (5,5),sticky="news")

        app.grip_feedback_pos = customtkinter.CTkLabel(app.Gripper_frame.content_frame, text="Gripper position feedback is: " + str(Gripper_data_in[1]), font=customtkinter.CTkFont(size=text_size))
        app.grip_feedback_pos.grid(row=11, column=0, padx=20,pady = (10,20),sticky="news")

        app.grip_feedback_current = customtkinter.CTkLabel(app.Gripper_frame.content_frame, text="Gripper current feedback is: " + str(Gripper_data_in[3]), font=customtkinter.CTkFont(size=text_size))
        app.grip_feedback_current.grid(row=12, column=0, padx=20,pady = (10,20),sticky="news")
    
        app.grip_object_detection = customtkinter.CTkLabel(app.Gripper_frame.content_frame, text="Gripper detected " + str(Gripper_data_in[4]), font=customtkinter.CTkFont(size=text_size))
        app.grip_object_detection.grid(row=13, column=0, padx=20,pady = (10,20),sticky="news")

        app.grip_object_size = customtkinter.CTkLabel(app.Gripper_frame.content_frame, text="Detected object size is:  " , font=customtkinter.CTkFont(size=text_size))
        app.grip_object_size.grid(row=14, column=0, padx=20,pady = (10,20),sticky="news")

    def _entry(parent, value="", width=90):
        widget = customtkinter.CTkEntry(parent, width=width)
        widget.insert(0, str(value))
        return widget

    def _entry_value(entry, default="", cast=str):
        try:
            text = entry.get().strip()
            if text == "":
                return default
            return cast(text)
        except Exception:
            return default

    def _grid_labeled_entry(parent, row, column, label, value="", width=90, cast=str, store=None):
        customtkinter.CTkLabel(parent, text=label, anchor="w").grid(row=row, column=column, padx=6, pady=4, sticky="w")
        entry = _entry(parent, value, width=width)
        entry.grid(row=row, column=column + 1, padx=6, pady=4, sticky="we")
        if store is not None:
            store[label] = (entry, cast)
        return entry

    def vision_frame():
        cfg_all = load_config()
        cfg = cfg_all["vision"]
        workspace = cfg_all["workspace"]

        title = customtkinter.CTkLabel(app.vision_frame, text="Vision System", font=customtkinter.CTkFont(size=20, weight="bold"))
        title.grid(row=0, column=0, columnspan=3, padx=16, pady=(12, 6), sticky="w")

        live_panel = customtkinter.CTkFrame(app.vision_frame, corner_radius=0)
        live_panel.grid(row=1, column=0, padx=(12, 2), pady=(0, 12), sticky="nsew")
        live_panel.grid_columnconfigure(0, weight=1)
        live_panel.grid_rowconfigure(1, weight=1)

        # Draggable Divider Handle
        app.vision_handle = customtkinter.CTkFrame(app.vision_frame, width=7, corner_radius=3, fg_color=UI_HANDLE, cursor="sb_h_double_arrow")
        app.vision_handle.grid(row=1, column=1, sticky="ns", padx=2, pady=12)

        def start_vision_resize(event):
            app.vision_drag_start_x = event.x_root
            app.vision_drag_start_width = app.vision_left_width

        def drag_vision_resize(event):
            delta = event.x_root - app.vision_drag_start_x
            new_width = max(300, min(1400, app.vision_drag_start_width + delta))
            app.vision_left_width = new_width
            app.vision_frame.grid_columnconfigure(0, minsize=new_width)

        app.vision_handle.bind("<ButtonPress-1>", start_vision_resize)
        app.vision_handle.bind("<B1-Motion>", drag_vision_resize)

        controls = customtkinter.CTkFrame(live_panel, corner_radius=0)
        controls.grid(row=0, column=0, padx=8, pady=8, sticky="ew")
        controls.grid_columnconfigure(1, weight=1)
        customtkinter.CTkLabel(controls, text="Source").grid(row=0, column=0, padx=6, pady=4, sticky="w")
        app.vision_source_menu = customtkinter.CTkOptionMenu(controls, values=vision_manager.available_sources())
        app.vision_source_menu.set(str(cfg.get("video_source", "MOCK")))
        app.vision_source_menu.grid(row=0, column=1, padx=6, pady=4, sticky="we")
        app.vision_refresh_sources = customtkinter.CTkButton(controls, text="Refresh", width=70, command=refresh_vision_sources)
        app.vision_refresh_sources.grid(row=0, column=2, padx=6, pady=4)
        app.vision_start = customtkinter.CTkButton(controls, text="Start Camera", command=start_vision_camera)
        app.vision_start.grid(row=0, column=3, padx=6, pady=4)
        app.vision_stop = customtkinter.CTkButton(controls, text="Stop Camera", command=stop_vision_camera)
        app.vision_stop.grid(row=0, column=4, padx=6, pady=4)

        app.vision_feed_label = customtkinter.CTkLabel(live_panel, text="Camera feed not started", anchor="center")
        app.vision_feed_label.grid(row=1, column=0, padx=8, pady=8, sticky="nsew")
        app.vision_safety_badge = customtkinter.CTkLabel(live_panel, text="UNKNOWN", height=28, anchor="center")
        app.vision_safety_badge.grid(row=2, column=0, padx=8, pady=(0, 8), sticky="ew")

        app.vision_marginal_frame = customtkinter.CTkFrame(live_panel, corner_radius=0)
        app.vision_marginal_frame.grid(row=3, column=0, padx=8, pady=(0, 8), sticky="ew")
        app.vision_marginal_frame.grid_columnconfigure(0, weight=1)
        app.vision_marginal_label = customtkinter.CTkLabel(app.vision_marginal_frame, text="MARGINAL pick waiting for confirmation", anchor="w")
        app.vision_marginal_label.grid(row=0, column=0, padx=8, pady=6, sticky="ew")
        customtkinter.CTkButton(app.vision_marginal_frame, text="Confirm Pick", width=110, command=lambda: set_marginal_decision("confirm")).grid(row=0, column=1, padx=6, pady=6)
        customtkinter.CTkButton(app.vision_marginal_frame, text="Skip", width=70, command=lambda: set_marginal_decision("skip")).grid(row=0, column=2, padx=6, pady=6)
        app.vision_marginal_countdown = customtkinter.CTkLabel(app.vision_marginal_frame, text="", anchor="w")
        app.vision_marginal_countdown.grid(row=1, column=0, columnspan=3, padx=8, pady=(0, 6), sticky="ew")
        app.vision_marginal_frame.grid_remove()

        side = customtkinter.CTkScrollableFrame(app.vision_frame, corner_radius=0)
        side.grid(row=1, column=2, padx=(2, 12), pady=(0, 12), sticky="nsew")
        side.grid_columnconfigure(0, weight=1)
        app.vision_entries = {}

        detection = CollapsibleFrame(side, title="Detection Setup")
        detection.grid(row=0, column=0, padx=4, pady=6, sticky="ew")
        detection.content_frame.grid_columnconfigure((1, 3), weight=1)
        customtkinter.CTkLabel(detection.content_frame, text="Method").grid(row=1, column=0, padx=6, pady=4, sticky="w")
        app.vision_method = customtkinter.CTkOptionMenu(detection.content_frame, values=["adaptive", "canny", "hsv", "model"])
        app.vision_method.set(str(cfg.get("detection_method", "adaptive")))
        app.vision_method.grid(row=1, column=1, padx=6, pady=4, sticky="we")
        app.vision_tune_live = customtkinter.CTkCheckBox(detection.content_frame, text="Tune Live")
        app.vision_tune_live.grid(row=1, column=2, columnspan=2, padx=6, pady=4, sticky="w")
        if bool(cfg.get("tune_live", False)):
            app.vision_tune_live.select()
        _grid_labeled_entry(detection.content_frame, 2, 0, "Adaptive block", cfg.get("adaptive_block_size", 11), store=app.vision_entries, cast=int)
        _grid_labeled_entry(detection.content_frame, 2, 2, "Adaptive C", cfg.get("adaptive_c", 2), store=app.vision_entries, cast=int)
        _grid_labeled_entry(detection.content_frame, 3, 0, "Canny t1", cfg.get("canny_threshold1", 50), store=app.vision_entries, cast=int)
        _grid_labeled_entry(detection.content_frame, 3, 2, "Canny t2", cfg.get("canny_threshold2", 150), store=app.vision_entries, cast=int)
        _grid_labeled_entry(detection.content_frame, 4, 0, "HSV lower", ",".join(str(x) for x in cfg.get("hsv_lower", [0, 0, 150])), width=130, store=app.vision_entries)
        _grid_labeled_entry(detection.content_frame, 4, 2, "HSV upper", ",".join(str(x) for x in cfg.get("hsv_upper", [180, 60, 255])), width=130, store=app.vision_entries)
        _grid_labeled_entry(detection.content_frame, 5, 0, "Min area", cfg.get("min_contour_area", 500), store=app.vision_entries, cast=int)
        _grid_labeled_entry(detection.content_frame, 5, 2, "Max area", cfg.get("max_contour_area", 50000), store=app.vision_entries, cast=int)
        _grid_labeled_entry(detection.content_frame, 6, 0, "Morph kernel", cfg.get("morph_kernel_size", 3), store=app.vision_entries, cast=int)
        _grid_labeled_entry(detection.content_frame, 6, 2, "Brightness", cfg.get("brightness", 0), store=app.vision_entries, cast=int)
        _grid_labeled_entry(detection.content_frame, 7, 0, "Contrast", cfg.get("contrast", 1.0), store=app.vision_entries, cast=float)
        _grid_labeled_entry(detection.content_frame, 7, 2, "Zoom", cfg.get("zoom", 1.0), store=app.vision_entries, cast=float)
        app.vision_apply = customtkinter.CTkButton(detection.content_frame, text="Apply Detection", command=lambda: save_vision_settings(False))
        app.vision_apply.grid(row=8, column=0, columnspan=2, padx=6, pady=8, sticky="we")
        app.vision_save = customtkinter.CTkButton(detection.content_frame, text="Save Detection", command=lambda: save_vision_settings(True))
        app.vision_save.grid(row=8, column=2, columnspan=2, padx=6, pady=8, sticky="we")

        model = CollapsibleFrame(side, title="Model Config")
        model.grid(row=1, column=0, padx=4, pady=6, sticky="ew")
        model.content_frame.grid_columnconfigure(1, weight=1)
        customtkinter.CTkLabel(model.content_frame, text="ONNX path").grid(row=1, column=0, padx=6, pady=4, sticky="w")
        app.vision_model_entry = _entry(model.content_frame, cfg.get("model_path", "vision/models/best.onnx"), width=300)
        app.vision_model_entry.grid(row=1, column=1, padx=6, pady=4, sticky="we")
        customtkinter.CTkButton(model.content_frame, text="Browse", width=70, command=browse_vision_model).grid(row=1, column=2, padx=6, pady=4)
        customtkinter.CTkButton(model.content_frame, text="Load", width=70, command=load_vision_model).grid(row=1, column=3, padx=6, pady=4)
        _grid_labeled_entry(model.content_frame, 2, 0, "Conf", cfg.get("model_conf_threshold", 0.5), store=app.vision_entries, cast=float)
        _grid_labeled_entry(model.content_frame, 2, 2, "IoU", cfg.get("model_iou_threshold", 0.45), store=app.vision_entries, cast=float)
        app.vision_model_status = customtkinter.CTkLabel(model.content_frame, text="Model: NOT LOADED", anchor="w")
        app.vision_model_status.grid(row=3, column=0, columnspan=4, padx=6, pady=4, sticky="we")

        info = CollapsibleFrame(side, title="Object Info")
        info.grid(row=2, column=0, padx=4, pady=6, sticky="ew")
        info.content_frame.grid_columnconfigure(1, weight=1)
        app.vision_info_labels = {}
        for idx, key in enumerate(["Status", "BBox", "Centroid", "Dimension", "Pick point", "Orientation"]):
            customtkinter.CTkLabel(info.content_frame, text=key).grid(row=idx + 1, column=0, padx=6, pady=3, sticky="w")
            label = customtkinter.CTkLabel(info.content_frame, text="-", anchor="w", justify="left")
            label.grid(row=idx + 1, column=1, padx=6, pady=3, sticky="we")
            app.vision_info_labels[key] = label

        pick_zone = CollapsibleFrame(side, title="Safe Pick Zone")
        pick_zone.grid(row=3, column=0, padx=4, pady=6, sticky="ew")
        pick_zone.content_frame.grid_columnconfigure((1, 3), weight=1)
        _grid_labeled_entry(pick_zone.content_frame, 1, 0, "Inset X", cfg.get("pick_inset_x_mm", 10.0), store=app.vision_entries, cast=float)
        _grid_labeled_entry(pick_zone.content_frame, 1, 2, "Inset Y", cfg.get("pick_inset_y_mm", 8.0), store=app.vision_entries, cast=float)
        _grid_labeled_entry(pick_zone.content_frame, 2, 0, "Safe margin", cfg.get("safe_pick_margin_pct", 0.15), store=app.vision_entries, cast=float)
        _grid_labeled_entry(pick_zone.content_frame, 2, 2, "Marginal timeout", cfg.get("marginal_confirm_timeout_s", 2.0), store=app.vision_entries, cast=float)
        app.vision_preview_pick_zone = customtkinter.CTkCheckBox(pick_zone.content_frame, text="Preview Pick Zone")
        app.vision_preview_pick_zone.grid(row=3, column=0, columnspan=4, padx=6, pady=4, sticky="w")
        if bool(cfg.get("preview_pick_zone", True)):
            app.vision_preview_pick_zone.select()

        workspace_panel = CollapsibleFrame(side, title="Workspace")
        workspace_panel.grid(row=4, column=0, padx=4, pady=6, sticky="ew")
        workspace_panel.content_frame.grid_columnconfigure((1, 3), weight=1)
        app.workspace_entries = {}
        _grid_labeled_entry(workspace_panel.content_frame, 1, 0, "X min", workspace.get("x_min_mm", -200), store=app.workspace_entries, cast=float)
        _grid_labeled_entry(workspace_panel.content_frame, 1, 2, "X max", workspace.get("x_max_mm", 200), store=app.workspace_entries, cast=float)
        _grid_labeled_entry(workspace_panel.content_frame, 2, 0, "Y min", workspace.get("y_min_mm", -200), store=app.workspace_entries, cast=float)
        _grid_labeled_entry(workspace_panel.content_frame, 2, 2, "Y max", workspace.get("y_max_mm", 200), store=app.workspace_entries, cast=float)
        _grid_labeled_entry(workspace_panel.content_frame, 3, 0, "Z fixed", workspace.get("z_fixed_mm", 200), store=app.workspace_entries, cast=float)
        _grid_labeled_entry(workspace_panel.content_frame, 3, 2, "Margin", workspace.get("margin_mm", 10), store=app.workspace_entries, cast=float)
        _grid_labeled_entry(workspace_panel.content_frame, 4, 0, "Offset X", cfg.get("offset_x_mm", 0), store=app.vision_entries, cast=float)
        _grid_labeled_entry(workspace_panel.content_frame, 4, 2, "Offset Y", cfg.get("offset_y_mm", 0), store=app.vision_entries, cast=float)

        auto_pick = CollapsibleFrame(side, title="Auto Pick")
        auto_pick.grid(row=5, column=0, padx=4, pady=6, sticky="ew")
        auto_pick.content_frame.grid_columnconfigure((1, 3), weight=1)
        _grid_labeled_entry(auto_pick.content_frame, 1, 0, "RPY deg", ",".join(str(x) for x in cfg.get("pick_pose_rpy_deg", [0, 0, 0])), width=130, store=app.vision_entries)
        _grid_labeled_entry(auto_pick.content_frame, 1, 2, "Pick time", cfg.get("pick_move_time_s", 4.0), store=app.vision_entries, cast=float)
        _grid_labeled_entry(auto_pick.content_frame, 2, 0, "Descent mm", cfg.get("post_pick_descent_mm", 50.0), store=app.vision_entries, cast=float)
        _grid_labeled_entry(auto_pick.content_frame, 2, 2, "Descent time", cfg.get("post_pick_move_time_s", 2.0), store=app.vision_entries, cast=float)
        _grid_labeled_entry(auto_pick.content_frame, 3, 0, "Gripper close", ",".join(str(x) for x in cfg.get("gripper_close", [255, 100, 120])), width=130, store=app.vision_entries)
        customtkinter.CTkButton(auto_pick.content_frame, text="Run Vision Pick (vision())", command=run_vision_pick_now).grid(row=4, column=0, columnspan=4, padx=6, pady=(8, 4), sticky="we")

        ibvs_panel = CollapsibleFrame(side, title="IBVS Visual Servoing")
        ibvs_panel.grid(row=6, column=0, padx=4, pady=6, sticky="ew")
        ibvs_panel.content_frame.grid_columnconfigure((1, 3), weight=1)
        app.vision_ibvs_enabled = customtkinter.CTkCheckBox(ibvs_panel.content_frame, text="Enable IBVS")
        app.vision_ibvs_enabled.grid(row=0, column=0, columnspan=2, padx=6, pady=4, sticky="w")
        if bool(cfg.get("ibvs_enabled", False)):
            app.vision_ibvs_enabled.select()
        app.vision_ibvs_dispatch = customtkinter.CTkCheckBox(ibvs_panel.content_frame, text="Dispatch to robot")
        app.vision_ibvs_dispatch.grid(row=0, column=2, columnspan=2, padx=6, pady=4, sticky="w")
        _grid_labeled_entry(ibvs_panel.content_frame, 1, 0, "Depth m", cfg.get("ibvs_depth_m", 0.5), store=app.vision_entries, cast=float)
        _grid_labeled_entry(ibvs_panel.content_frame, 1, 2, "Lambda", cfg.get("ibvs_lambda", 1.0), store=app.vision_entries, cast=float)
        _grid_labeled_entry(ibvs_panel.content_frame, 2, 0, "Deadband px", cfg.get("ibvs_deadband_px", 4.0), store=app.vision_entries, cast=float)
        _grid_labeled_entry(ibvs_panel.content_frame, 2, 2, "Max step mm", cfg.get("ibvs_max_step_mm", 20.0), store=app.vision_entries, cast=float)
        customtkinter.CTkButton(ibvs_panel.content_frame, text="Start IBVS", width=110, command=start_ibvs_servo).grid(row=3, column=0, padx=6, pady=8, sticky="we")
        customtkinter.CTkButton(ibvs_panel.content_frame, text="Stop IBVS", width=110, command=stop_ibvs_servo).grid(row=3, column=1, padx=6, pady=8, sticky="we")
        app.vision_ibvs_status = customtkinter.CTkLabel(ibvs_panel.content_frame, text="IBVS: idle", anchor="w", justify="left")
        app.vision_ibvs_status.grid(row=4, column=0, columnspan=4, padx=6, pady=4, sticky="we")

        calibration = CollapsibleFrame(side, title="Camera Calibration")
        calibration.grid(row=7, column=0, padx=4, pady=6, sticky="ew")
        calibration.content_frame.grid_columnconfigure((1, 3), weight=1)
        cal_cfg = cfg.get("calibration", {})
        _grid_labeled_entry(calibration.content_frame, 1, 0, "Chessboard", ",".join(str(x) for x in cal_cfg.get("chessboard_size", [9, 6])), width=120, store=app.vision_entries)
        _grid_labeled_entry(calibration.content_frame, 1, 2, "Square mm", cal_cfg.get("square_size_mm", 25.0), store=app.vision_entries, cast=float)
        customtkinter.CTkButton(calibration.content_frame, text="Snap Frame", command=capture_vision_snapshot).grid(row=2, column=0, padx=6, pady=6, sticky="we")
        customtkinter.CTkButton(calibration.content_frame, text="Run Calibration", command=run_vision_calibration).grid(row=2, column=1, padx=6, pady=6, sticky="we")
        customtkinter.CTkButton(calibration.content_frame, text="Mock Detection", command=set_mock_vision_detection).grid(row=2, column=2, padx=6, pady=6, sticky="we")
        app.vision_calibration = customtkinter.CTkLabel(calibration.content_frame, text="Calibration: -", anchor="w", justify="left")
        app.vision_calibration.grid(row=3, column=0, columnspan=4, padx=6, pady=4, sticky="we")
        app.vision_status = customtkinter.CTkLabel(side, text="Status: stopped", anchor="w", justify="left")
        app.vision_status.grid(row=8, column=0, padx=6, pady=8, sticky="we")

    def modbus_frame():
        cfg = load_config()["modbus"]
        title = customtkinter.CTkLabel(app.modbus_frame, text="Modbus TCP/IP Settings", font=customtkinter.CTkFont(size=20, weight="bold"))
        title.grid(row=0, column=0, columnspan=3, padx=16, pady=(12, 6), sticky="w")

        top = CollapsibleFrame(app.modbus_frame, title="Modbus Connection Setup")
        top.grid(row=1, column=0, columnspan=3, padx=12, pady=(0, 8), sticky="ew")
        top.content_frame.grid_columnconfigure((1, 3, 5, 7), weight=1)
        app.modbus_entries = {}
        for col, (label, key) in enumerate([("IP", "ip"), ("Port", "port"), ("Slave ID", "slave_id")]):
            customtkinter.CTkLabel(top.content_frame, text=label).grid(row=0, column=col * 2, padx=6, pady=6, sticky="w")
            entry = _entry(top.content_frame, cfg.get(key, ""), width=120)
            entry.grid(row=0, column=col * 2 + 1, padx=6, pady=6, sticky="we")
            app.modbus_entries[key] = entry
        app.modbus_connect = customtkinter.CTkButton(top.content_frame, text="Connect", width=80, command=start_modbus)
        app.modbus_connect.grid(row=0, column=6, padx=6, pady=6)
        app.modbus_disconnect = customtkinter.CTkButton(top.content_frame, text="Disconnect", width=90, command=stop_modbus)
        app.modbus_disconnect.grid(row=0, column=7, padx=6, pady=6)
        app.modbus_save = customtkinter.CTkButton(top.content_frame, text="Save Config", width=90, command=save_modbus_settings)
        app.modbus_save.grid(row=0, column=8, padx=6, pady=6)

        # Container for Block A and Block B tests (now placed higher at row 2 and vertically resizable)
        tests = customtkinter.CTkFrame(app.modbus_frame, corner_radius=0)
        tests.grid(row=2, column=0, columnspan=3, padx=12, pady=(0, 6), sticky="nsew")
        tests.grid_columnconfigure((0, 1), weight=1)
        tests.grid_rowconfigure(0, weight=1)

        block_a_cfg = cfg.get("block_a", {})
        block_a = CollapsibleFrame(tests, title="Blok A - Protocol Performance Test")
        block_a.grid(row=0, column=0, padx=(0, 6), pady=0, sticky="nsew")
        block_a.content_frame.grid_columnconfigure((1, 3, 5, 7), weight=1)
        app.modbus_block_a_entries = {}
        for col, (label, key, default) in enumerate([
            ("IP", "ip", cfg.get("ip", "MOCK")),
            ("Port", "port", cfg.get("port", 502)),
            ("Slave", "slave_id", cfg.get("slave_id", 1)),
            ("Timeout ms", "timeout_ms", 1000),
        ]):
            customtkinter.CTkLabel(block_a.content_frame, text=label).grid(row=1, column=col * 2, padx=5, pady=3, sticky="w")
            entry = _entry(block_a.content_frame, block_a_cfg.get(key, default), width=90)
            entry.grid(row=1, column=col * 2 + 1, padx=5, pady=3, sticky="we")
            app.modbus_block_a_entries[key] = entry
        customtkinter.CTkLabel(block_a.content_frame, text="Function").grid(row=2, column=0, padx=5, pady=3, sticky="w")
        app.modbus_block_a_function = customtkinter.CTkOptionMenu(
            block_a.content_frame,
            values=["read_holding_register", "read_input_register", "read_coil", "read_discrete_input", "write_register", "write_coil"],
        )
        app.modbus_block_a_function.set(str(block_a_cfg.get("function", "read_holding_register")))
        app.modbus_block_a_function.grid(row=2, column=1, columnspan=3, padx=5, pady=3, sticky="we")
        for col, (label, key, default) in enumerate([("Address", "address", 100), ("Count", "count", 1), ("N", "iterations", 1000)], start=2):
            customtkinter.CTkLabel(block_a.content_frame, text=label).grid(row=2, column=col * 2, padx=5, pady=3, sticky="w")
            entry = _entry(block_a.content_frame, block_a_cfg.get(key, default), width=80)
            entry.grid(row=2, column=col * 2 + 1, padx=5, pady=3, sticky="we")
            app.modbus_block_a_entries[key] = entry
        customtkinter.CTkButton(block_a.content_frame, text="Start Block A", width=110, command=start_modbus_block_a).grid(row=3, column=0, columnspan=2, padx=5, pady=6, sticky="we")
        customtkinter.CTkButton(block_a.content_frame, text="Stop", width=70, command=stop_modbus_block_a).grid(row=3, column=2, padx=5, pady=6, sticky="we")
        app.modbus_block_a_labels = {}
        for idx, key in enumerate(["progress", "avg_rt_ms", "throughput_rps", "packet_loss_pct", "success_count", "failed_count", "timeout_count"]):
            label = customtkinter.CTkLabel(block_a.content_frame, text=f"{key}: -", anchor="w")
            label.grid(row=4 + idx // 4, column=(idx % 4) * 2, columnspan=2, padx=5, pady=2, sticky="we")
            app.modbus_block_a_labels[key] = label

        block_b_cfg = cfg.get("block_b", {})
        block_b = CollapsibleFrame(tests, title="Blok B - Cycle Time Test")
        block_b.grid(row=0, column=1, padx=(6, 0), pady=0, sticky="nsew")
        block_b.content_frame.grid_columnconfigure((1, 3, 5, 7), weight=1)
        app.modbus_block_b_entries = {}
        for col, (label, key, default) in enumerate([
            ("N", "iterations", 100),
            ("Trigger", "trigger_name", "trigger_pick"),
            ("Done", "done_name", "cycle_done"),
            ("Error", "error_name", "error_flag"),
        ]):
            customtkinter.CTkLabel(block_b.content_frame, text=label).grid(row=1, column=col * 2, padx=5, pady=3, sticky="w")
            entry = _entry(block_b.content_frame, block_b_cfg.get(key, default), width=90)
            entry.grid(row=1, column=col * 2 + 1, padx=5, pady=3, sticky="we")
            app.modbus_block_b_entries[key] = entry
        customtkinter.CTkButton(block_b.content_frame, text="Start Block B", width=110, command=start_modbus_block_b).grid(row=2, column=0, columnspan=2, padx=5, pady=6, sticky="we")
        customtkinter.CTkButton(block_b.content_frame, text="Stop", width=70, command=stop_modbus_block_b).grid(row=2, column=2, padx=5, pady=6, sticky="we")
        app.modbus_block_b_labels = {}
        for idx, key in enumerate(["progress", "avg_s", "min_s", "max_s", "std_s", "success_count", "failed_count", "active"]):
            label = customtkinter.CTkLabel(block_b.content_frame, text=f"{key}: -", anchor="w")
            label.grid(row=3 + idx // 4, column=(idx % 4) * 2, columnspan=2, padx=5, pady=2, sticky="we")
            app.modbus_block_b_labels[key] = label

        # Draggable horizontal divider handle
        app.modbus_h_handle = customtkinter.CTkFrame(app.modbus_frame, height=7, corner_radius=3, fg_color=UI_HANDLE, cursor="sb_v_double_arrow")
        app.modbus_h_handle.grid(row=3, column=0, columnspan=3, sticky="ew", padx=12, pady=2)

        def start_modbus_h_resize(event):
            app.modbus_drag_start_y = event.y_root
            app.modbus_drag_start_height = app.modbus_top_height

        def drag_modbus_h_resize(event):
            delta = event.y_root - app.modbus_drag_start_y
            new_height = max(200, min(800, app.modbus_drag_start_height + delta))
            app.modbus_top_height = new_height
            app.modbus_frame.grid_rowconfigure(2, minsize=new_height)

        app.modbus_h_handle.bind("<ButtonPress-1>", start_modbus_h_resize)
        app.modbus_h_handle.bind("<B1-Motion>", drag_modbus_h_resize)

        # Address Mapping (placed at row 4, stretched vertically)
        mapping = CollapsibleFrame(app.modbus_frame, title="Address Mapping")
        mapping.grid(row=4, column=0, padx=(12, 2), pady=(0, 12), sticky="nsew")
        mapping.content_frame.grid_columnconfigure(0, weight=1)
        mapping.content_frame.grid_rowconfigure(2, weight=1)
        
        toolbar = customtkinter.CTkFrame(mapping.content_frame, corner_radius=0)
        toolbar.grid(row=1, column=0, padx=8, pady=(0, 6), sticky="ew")
        customtkinter.CTkButton(toolbar, text="Add Row", width=80, command=add_modbus_row).grid(row=0, column=0, padx=4, pady=4)
        customtkinter.CTkButton(toolbar, text="Remove Row", width=100, command=remove_modbus_row).grid(row=0, column=1, padx=4, pady=4)
        columns = ("name", "type", "address", "rw", "desc")
        app.modbus_table = ttk.Treeview(mapping.content_frame, columns=columns, show="headings", height=14)
        headings = {"name": "Name", "type": "Type", "address": "Address", "rw": "RW", "desc": "Description"}
        widths = {"name": 130, "type": 110, "address": 80, "rw": 80, "desc": 260}
        for column in columns:
            app.modbus_table.heading(column, text=headings[column])
            app.modbus_table.column(column, width=widths[column], stretch=column == "desc")
        app.modbus_table.grid(row=2, column=0, padx=8, pady=(0, 8), sticky="nsew")
        app.modbus_table.bind("<Double-1>", edit_modbus_cell)
        for entry in cfg.get("addresses", []):
            app.modbus_table.insert("", "end", values=(entry.get("name",""), entry.get("type","coil"), entry.get("address",0), entry.get("rw","read"), entry.get("desc","")))

        # Draggable vertical divider handle
        app.modbus_v_handle = customtkinter.CTkFrame(app.modbus_frame, width=7, corner_radius=3, fg_color=UI_HANDLE, cursor="sb_h_double_arrow")
        app.modbus_v_handle.grid(row=4, column=1, sticky="ns", padx=2, pady=6)

        def start_modbus_v_resize(event):
            app.modbus_drag_start_x = event.x_root
            app.modbus_drag_start_width = app.modbus_left_width

        def drag_modbus_v_resize(event):
            delta = event.x_root - app.modbus_drag_start_x
            new_width = max(300, min(1400, app.modbus_drag_start_width + delta))
            app.modbus_left_width = new_width
            app.modbus_frame.grid_columnconfigure(0, minsize=new_width)

        app.modbus_v_handle.bind("<ButtonPress-1>", start_modbus_v_resize)
        app.modbus_v_handle.bind("<B1-Motion>", drag_modbus_v_resize)

        # Live Monitor (placed at row 4, stretched vertically)
        monitor = CollapsibleFrame(app.modbus_frame, title="Live Monitor & Jog Controls")
        monitor.grid(row=4, column=2, padx=(2, 12), pady=(0, 12), sticky="nsew")
        monitor.content_frame.grid_columnconfigure(0, weight=1)
        monitor.content_frame.grid_rowconfigure(3, weight=1)
        
        app.modbus_status = customtkinter.CTkLabel(monitor.content_frame, text="Status: disconnected", anchor="w")
        app.modbus_status.grid(row=1, column=0, padx=8, pady=4, sticky="we")
        app.modbus_jog_source = customtkinter.CTkLabel(monitor.content_frame, text="Active jog source: NONE", anchor="w")
        app.modbus_jog_source.grid(row=2, column=0, padx=8, pady=4, sticky="we")
        app.modbus_monitor_table = ttk.Treeview(monitor.content_frame, columns=("signal", "value"), show="headings", height=12)
        app.modbus_monitor_table.heading("signal", text="Signal")
        app.modbus_monitor_table.heading("value", text="Value")
        app.modbus_monitor_table.column("signal", width=170)
        app.modbus_monitor_table.column("value", width=120)
        app.modbus_monitor_table.grid(row=3, column=0, padx=8, pady=8, sticky="nsew")

        jog = customtkinter.CTkFrame(monitor.content_frame, corner_radius=0)
        jog.grid(row=4, column=0, padx=8, pady=8, sticky="ew")
        jog.grid_columnconfigure((1, 3), weight=1)
        customtkinter.CTkLabel(jog, text="Jog speed %").grid(row=0, column=0, padx=6, pady=4, sticky="w")
        app.modbus_jog_speed = _entry(jog, cfg.get("jog_speed_pct", 20), width=80)
        app.modbus_jog_speed.grid(row=0, column=1, padx=6, pady=4, sticky="we")
        customtkinter.CTkLabel(jog, text="Lock timeout s").grid(row=0, column=2, padx=6, pady=4, sticky="w")
        app.modbus_jog_timeout = _entry(jog, cfg.get("jog_lock_timeout_s", 5.0), width=80)
        app.modbus_jog_timeout.grid(row=0, column=3, padx=6, pady=4, sticky="we")

    def research_frame():
        title = customtkinter.CTkLabel(app.research_frame, text="Research Logger", font=customtkinter.CTkFont(size=20, weight="bold"))
        title.grid(row=0, column=0, columnspan=3, padx=16, pady=(12, 6), sticky="w")
        
        controls = CollapsibleFrame(app.research_frame, title="Logger Session Controls")
        controls.grid(row=1, column=0, columnspan=3, padx=12, pady=(0, 8), sticky="ew")
        controls.content_frame.grid_columnconfigure(5, weight=1)
        
        app.research_status = customtkinter.CTkLabel(controls.content_frame, text="IDLE", width=90)
        app.research_status.grid(row=0, column=0, padx=6, pady=6)
        app.research_elapsed = customtkinter.CTkLabel(controls.content_frame, text="00:00", width=80)
        app.research_elapsed.grid(row=0, column=1, padx=6, pady=6)
        customtkinter.CTkButton(controls.content_frame, text="Start Recording", width=120, command=lambda: set_research_enabled(True)).grid(row=0, column=2, padx=6, pady=6)
        customtkinter.CTkButton(controls.content_frame, text="Stop", width=70, command=lambda: set_research_enabled(False)).grid(row=0, column=3, padx=6, pady=6)
        customtkinter.CTkButton(controls.content_frame, text="Reset", width=70, command=reset_research).grid(row=0, column=4, padx=6, pady=6)
        customtkinter.CTkButton(controls.content_frame, text="Export CSV/Excel", width=130, command=export_research_csv).grid(row=0, column=6, padx=6, pady=6)
        customtkinter.CTkButton(controls.content_frame, text="Mark Event", width=100, command=lambda: research_logger.record("manual_event", 1, "UI marker")).grid(row=0, column=7, padx=6, pady=6)

        kpis = CollapsibleFrame(app.research_frame, title="KPI Metrics & Performance Graphs")
        kpis.grid(row=2, column=0, padx=(12, 2), pady=(0, 6), sticky="nsew")
        kpis.content_frame.grid_columnconfigure((0,1,2,3,4,5), weight=1)
        
        app.research_kpi_labels = {}
        for idx, key in enumerate(["serial_latency", "modbus_cycle", "vision_time", "pick_success_rate", "cycle_time", "timestamp_duration_s"]):
            card = customtkinter.CTkFrame(kpis.content_frame, corner_radius=0)
            card.grid(row=0, column=idx, padx=5, pady=6, sticky="ew")
            customtkinter.CTkLabel(card, text=key.replace("_", " ").title()).pack(padx=8, pady=(8, 2))
            value = customtkinter.CTkLabel(card, text="--", font=customtkinter.CTkFont(size=22, weight="bold"))
            value.pack(padx=8, pady=(2, 8))
            app.research_kpi_labels[key] = value
            
        app.research_plot_fig = plt.Figure(figsize=(7, 4), dpi=100)
        app.research_plot_fig.patch.set_facecolor(UI_SURFACE)
        app.research_axes = {}
        for index, key in enumerate(["serial_latency", "modbus_cycle", "vision_time", "cycle_time", "timestamp_duration_s", "modbus_block_b_cycle_time_s"], start=1):
            axis = app.research_plot_fig.add_subplot(3, 2, index)
            axis.set_facecolor(UI_SURFACE_LOW)
            axis.set_title(key.replace("_", " ").title(), color=UI_ON_SURFACE, fontsize=8, pad=4)
            axis.grid(True, alpha=0.25, color=UI_ON_SURFACE_MUTE)
            axis.tick_params(axis='both', which='major', labelsize=7, colors=UI_ON_SURFACE_MUTE)
            for spine in ['top', 'bottom', 'left', 'right']:
                axis.spines[spine].set_color(UI_BORDER)
            app.research_axes[key] = axis
        app.research_plot_fig.tight_layout()
        
        app.research_plot_canvas = FigureCanvasTkAgg(app.research_plot_fig, master=kpis.content_frame)
        app.research_plot_canvas.get_tk_widget().grid(row=1, column=0, columnspan=6, padx=8, pady=8, sticky="nsew")
        kpis.content_frame.grid_rowconfigure(1, weight=1)

        # Draggable vertical divider handle
        app.research_v_handle = customtkinter.CTkFrame(app.research_frame, width=7, corner_radius=3, fg_color=UI_HANDLE, cursor="sb_h_double_arrow")
        app.research_v_handle.grid(row=2, column=1, sticky="ns", padx=2, pady=6)

        def start_research_v_resize(event):
            app.research_drag_start_x = event.x_root
            app.research_drag_start_width = app.research_left_width

        def drag_research_v_resize(event):
            delta = event.x_root - app.research_drag_start_x
            new_width = max(300, min(1400, app.research_drag_start_width + delta))
            app.research_left_width = new_width
            app.research_frame.grid_columnconfigure(0, minsize=new_width)

        app.research_v_handle.bind("<ButtonPress-1>", start_research_v_resize)
        app.research_v_handle.bind("<B1-Motion>", drag_research_v_resize)

        events = CollapsibleFrame(app.research_frame, title="Failsafe Event Log")
        events.grid(row=2, column=2, padx=(2, 12), pady=(0, 6), sticky="nsew")
        events.content_frame.grid_columnconfigure(0, weight=1)
        events.content_frame.grid_rowconfigure(2, weight=1)
        
        app.research_path = customtkinter.CTkLabel(events.content_frame, text=f"File: {research_logger.path}", anchor="w", justify="left")
        app.research_path.grid(row=1, column=0, padx=8, pady=4, sticky="we")
        app.research_events = customtkinter.CTkTextbox(events.content_frame, font=customtkinter.CTkFont(size=12, family='TkDefaultFont'))
        app.research_events.grid(row=2, column=0, padx=8, pady=8, sticky="nsew")
        app.research_last = customtkinter.CTkLabel(events.content_frame, text="Last event: none", anchor="w", justify="left")
        app.research_last.grid(row=3, column=0, padx=8, pady=4, sticky="we")

        # Draggable horizontal divider handle
        app.research_h_handle = customtkinter.CTkFrame(app.research_frame, height=7, corner_radius=3, fg_color=UI_HANDLE, cursor="sb_v_double_arrow")
        app.research_h_handle.grid(row=3, column=0, columnspan=3, sticky="ew", padx=12, pady=2)

        def start_research_h_resize(event):
            app.research_drag_start_y = event.y_root
            app.research_drag_start_height = app.research_top_height

        def drag_research_h_resize(event):
            delta = event.y_root - app.research_drag_start_y
            new_height = max(200, min(800, app.research_drag_start_height + delta))
            app.research_top_height = new_height
            app.research_frame.grid_rowconfigure(2, minsize=new_height)

        app.research_h_handle.bind("<ButtonPress-1>", start_research_h_resize)
        app.research_h_handle.bind("<B1-Motion>", drag_research_h_resize)

        timestamp_panel = CollapsibleFrame(app.research_frame, title="Timestamp Table")
        timestamp_panel.grid(row=4, column=0, columnspan=3, padx=12, pady=(0, 12), sticky="nsew")
        timestamp_panel.content_frame.grid_columnconfigure(0, weight=1)
        timestamp_panel.content_frame.grid_rowconfigure(1, weight=1)
        
        timestamp_header = customtkinter.CTkFrame(timestamp_panel.content_frame, corner_radius=0)
        timestamp_header.grid(row=0, column=0, columnspan=2, padx=8, pady=(6, 2), sticky="ew")
        timestamp_header.grid_columnconfigure(0, weight=1)
        customtkinter.CTkLabel(timestamp_header, text="Timestamp Table", font=customtkinter.CTkFont(size=15, weight="bold")).grid(row=0, column=0, sticky="w")
        customtkinter.CTkButton(timestamp_header, text="Reset Table", width=120, command=reset_timestamp_table).grid(row=0, column=1, padx=5)
        customtkinter.CTkButton(timestamp_header, text="Export Timestamp", width=130, command=export_timestamp_table).grid(row=0, column=2, padx=5)

        timestamp_columns = ("time", "event", "label", "elapsed_s", "duration_s")
        app.timestamp_tree = ttk.Treeview(timestamp_panel.content_frame, columns=timestamp_columns, show="headings", height=8)
        app.timestamp_tree.heading("time", text="Waktu")
        app.timestamp_tree.heading("event", text="Event")
        app.timestamp_tree.heading("label", text="Label Program")
        app.timestamp_tree.heading("elapsed_s", text="Elapsed s")
        app.timestamp_tree.heading("duration_s", text="Duration s")
        app.timestamp_tree.column("time", width=190, anchor="w")
        app.timestamp_tree.column("event", width=90, anchor="center")
        app.timestamp_tree.column("label", width=300, anchor="w")
        app.timestamp_tree.column("elapsed_s", width=90, anchor="e")
        app.timestamp_tree.column("duration_s", width=90, anchor="e")
        app.timestamp_tree.grid(row=1, column=0, padx=(8, 0), pady=(2, 8), sticky="nsew")
        app.timestamp_tree_scroll = ttk.Scrollbar(timestamp_panel.content_frame, orient="vertical", command=app.timestamp_tree.yview)
        app.timestamp_tree_scroll.grid(row=1, column=1, padx=(0, 8), pady=(2, 8), sticky="ns")
        app.timestamp_tree.configure(yscrollcommand=app.timestamp_tree_scroll.set)

    def _parse_int_list(text, default):
        try:
            values = [int(float(part.strip())) for part in str(text).split(",") if part.strip()]
            return values if values else default
        except Exception:
            return default

    def _parse_float_list(text, default):
        try:
            values = [float(part.strip()) for part in str(text).split(",") if part.strip()]
            return values if values else default
        except Exception:
            return default

    def build_vision_settings():
        entries = getattr(app, "vision_entries", {})
        settings = {
            "video_source": app.vision_source_menu.get(),
            "detection_method": app.vision_method.get(),
            "tune_live": bool(app.vision_tune_live.get()),
            "preview_pick_zone": bool(app.vision_preview_pick_zone.get()),
            "model_path": app.vision_model_entry.get().strip(),
            "ibvs_enabled": bool(getattr(app, "vision_ibvs_enabled", None) and app.vision_ibvs_enabled.get()),
        }
        key_map = {
            "Adaptive block": "adaptive_block_size",
            "Adaptive C": "adaptive_c",
            "Canny t1": "canny_threshold1",
            "Canny t2": "canny_threshold2",
            "Min area": "min_contour_area",
            "Max area": "max_contour_area",
            "Morph kernel": "morph_kernel_size",
            "Brightness": "brightness",
            "Contrast": "contrast",
            "Zoom": "zoom",
            "Conf": "model_conf_threshold",
            "IoU": "model_iou_threshold",
            "Inset X": "pick_inset_x_mm",
            "Inset Y": "pick_inset_y_mm",
            "Safe margin": "safe_pick_margin_pct",
            "Marginal timeout": "marginal_confirm_timeout_s",
            "Offset X": "offset_x_mm",
            "Offset Y": "offset_y_mm",
            "Pick time": "pick_move_time_s",
            "Descent mm": "post_pick_descent_mm",
            "Descent time": "post_pick_move_time_s",
            "Square mm": "calibration.square_size_mm",
            "Depth m": "ibvs_depth_m",
            "Lambda": "ibvs_lambda",
            "Deadband px": "ibvs_deadband_px",
            "Max step mm": "ibvs_max_step_mm",
        }
        for label, key in key_map.items():
            if label in entries:
                entry, cast = entries[label]
                value = _entry_value(entry, load_config()["vision"].get(key.split(".")[-1], ""), cast)
                if key.startswith("calibration."):
                    settings.setdefault("calibration", {})[key.split(".", 1)[1]] = value
                else:
                    settings[key] = value
        settings["hsv_lower"] = _parse_int_list(entries["HSV lower"][0].get(), [0, 0, 150])
        settings["hsv_upper"] = _parse_int_list(entries["HSV upper"][0].get(), [180, 60, 255])
        settings["pick_pose_rpy_deg"] = _parse_float_list(entries["RPY deg"][0].get(), [0.0, 0.0, 0.0])[:3]
        settings["gripper_close"] = _parse_int_list(entries["Gripper close"][0].get(), [255, 100, 120])[:3]
        settings["calibration"]["chessboard_size"] = _parse_int_list(entries["Chessboard"][0].get(), [9, 6])[:2]
        return settings

    def build_workspace_settings():
        workspace = {}
        key_map = {"X min": "x_min_mm", "X max": "x_max_mm", "Y min": "y_min_mm", "Y max": "y_max_mm", "Z fixed": "z_fixed_mm", "Margin": "margin_mm"}
        for label, key in key_map.items():
            entry, cast = app.workspace_entries[label]
            workspace[key] = _entry_value(entry, load_config()["workspace"].get(key, 0), cast)
        return workspace

    def save_vision_settings(persist=True):
        settings = build_vision_settings()
        workspace = build_workspace_settings()
        if persist:
            cfg = load_config()
            calibration_update = settings.pop("calibration", {})
            cfg["vision"].update(settings)
            cfg["vision"].setdefault("calibration", {}).update(calibration_update)
            cfg["workspace"].update(workspace)
            save_config(cfg)
        else:
            calibration_update = settings.pop("calibration", {})
            if calibration_update:
                cfg = load_config()
                cfg["vision"].setdefault("calibration", {}).update(calibration_update)
                save_config(cfg)
        vision_manager.save_settings(settings, workspace)
        research_logger.record("vision_settings_saved", 1, "persist" if persist else "live")
        shared_string.value = b'Log: Vision settings saved'

    def refresh_vision_sources():
        sources = vision_manager.available_sources()
        app.vision_source_menu.configure(values=sources)
        if app.vision_source_menu.get() not in sources:
            app.vision_source_menu.set(sources[0])

    def browse_vision_model():
        path = filedialog.askopenfilename(title="Select ONNX model", filetypes=(("ONNX Model", "*.onnx"), ("All Files", "*.*")))
        if path:
            app.vision_model_entry.delete(0, tk.END)
            app.vision_model_entry.insert(0, path)

    def load_vision_model():
        save_vision_settings(True)
        status = vision_manager.load_model(app.vision_model_entry.get().strip())
        app.vision_model_status.configure(text="Model: " + str(status))
        research_logger.record("vision_model_load", 1 if status == "LOADED" else 0, status)

    def start_vision_camera():
        save_vision_settings(True)
        source = app.vision_source_menu.get()
        vision_manager.start(source)
        research_logger.record("vision_start", 1, source)

    def stop_vision_camera():
        vision_manager.stop()
        research_logger.record("vision_stop", 1)

    def set_mock_vision_detection():
        vision_manager.set_mock_detection(0.0, 0.0)
        research_logger.record("vision_mock_detection", 1)

    def set_marginal_decision(decision):
        state = read_state("vision_confirmation", {})
        token = state.get("token", time.time()) if isinstance(state, dict) else time.time()
        update_state("vision_confirmation", {"status": decision, "token": token, "decision": decision, "updated_at": time.time()})
        research_logger.record("marginal_" + decision, 1)

    def capture_vision_snapshot():
        try:
            count = vision_manager.capture_snapshot()
            shared_string.value = f"Log: Calibration snapshot {count}".encode("utf-8")[:99]
            research_logger.record("vision_calibration_snapshot", count)
        except Exception as exc:
            shared_string.value = f"Error: Snapshot failed {exc}".encode("utf-8")[:99]

    def run_vision_calibration():
        settings = build_vision_settings()
        cal = settings.get("calibration", {})
        chessboard = tuple(cal.get("chessboard_size", [9, 6])[:2])
        square = float(cal.get("square_size_mm", 25.0))
        try:
            summary = vision_manager.run_calibration(chessboard, square)
            shared_string.value = b'Log: Vision calibration complete'
            research_logger.record("vision_calibration", 1, str(summary))
        except Exception as exc:
            shared_string.value = f"Error: Calibration failed {exc}".encode("utf-8")[:99]
            research_logger.record("vision_calibration_error", 1, str(exc))

    def run_vision_pick_now():
        save_vision_settings(True)
        try:
            sequence = build_vision_pick_sequence(shared_string)
        except Exception as exc:
            shared_string.value = f"Error: vision pick failed {exc}".encode("utf-8")[:99]
            research_logger.record("vision_pick_error", 1, str(exc))
            return
        if not sequence:
            research_logger.record("vision_pick_dry_run", 0, "no sequence")
            return
        try:
            shared_string.value = ("Log: vision pick sequence: " + " | ".join(sequence)).encode("utf-8")[:99]
        except Exception:
            pass
        research_logger.record("vision_pick_dry_run", len(sequence), "|".join(sequence))

    def start_ibvs_servo():
        save_vision_settings(True)
        dispatch = bool(app.vision_ibvs_dispatch.get())
        ipc = {"shared_string": shared_string}
        try:
            status = vision_manager.start_ibvs(ipc_arrays=ipc, dispatch=dispatch)
            app.vision_ibvs_status.configure(text=f"IBVS: started (dispatch={dispatch})")
            research_logger.record("ibvs_start", 1, str(status))
        except Exception as exc:
            app.vision_ibvs_status.configure(text=f"IBVS: error {exc}")
            research_logger.record("ibvs_start_error", 1, str(exc))

    def stop_ibvs_servo():
        try:
            vision_manager.stop_ibvs()
            app.vision_ibvs_status.configure(text="IBVS: stopped")
            research_logger.record("ibvs_stop", 1)
        except Exception as exc:
            app.vision_ibvs_status.configure(text=f"IBVS: error {exc}")

    def add_modbus_row(row=None):
        row = row or {"name": "signal", "type": "coil", "address": 0, "rw": "read", "desc": ""}
        app.modbus_table.insert("", "end", values=(row.get("name", "signal"), row.get("type", "coil"), row.get("address", 0), row.get("rw", "read"), row.get("desc", "")))

    def remove_modbus_row():
        for item in app.modbus_table.selection():
            app.modbus_table.delete(item)

    def edit_modbus_cell(event):
        region = app.modbus_table.identify("region", event.x, event.y)
        if region != "cell":
            return
        row_id = app.modbus_table.identify_row(event.y)
        column = app.modbus_table.identify_column(event.x)
        if not row_id or not column:
            return
        column_index = int(column.replace("#", "")) - 1
        bbox = app.modbus_table.bbox(row_id, column)
        values = list(app.modbus_table.item(row_id, "values"))
        edit = tk.Entry(app.modbus_table)
        edit.insert(0, values[column_index])
        edit.place(x=bbox[0], y=bbox[1], width=bbox[2], height=bbox[3])
        edit.focus()
        def commit(_event=None):
            values[column_index] = edit.get()
            app.modbus_table.item(row_id, values=values)
            edit.destroy()
        edit.bind("<Return>", commit)
        edit.bind("<FocusOut>", commit)

    def _parse_modbus_address_rows():
        rows = []
        for item in app.modbus_table.get_children():
            values = app.modbus_table.item(item, "values")
            if not values or not values[0]:
                continue
            rows.append({
                "name": str(values[0]).strip(),
                "type": str(values[1] or "coil").strip(),
                "address": int(float(values[2] or 0)),
                "rw": str(values[3] or "read").strip(),
                "desc": str(values[4] if len(values) > 4 else ""),
            })
        return rows

    def save_modbus_settings():
        modbus_manager.save_config(
            app.modbus_entries["ip"].get(),
            int(float(app.modbus_entries["port"].get() or 502)),
            int(float(app.modbus_entries["slave_id"].get() or 1)),
            _parse_modbus_address_rows(),
            _entry_value(app.modbus_jog_speed, 20, float),
            _entry_value(app.modbus_jog_timeout, 5.0, float),
        )
        if hasattr(app, "modbus_block_a_entries"):
            _save_modbus_block_settings()
        research_logger.record("modbus_settings_saved", 1)
        shared_string.value = b'Log: Modbus settings saved'

    def start_modbus():
        save_modbus_settings()
        modbus_manager.start()
        research_logger.record("modbus_start", 1)

    def stop_modbus():
        modbus_manager.stop()
        research_logger.record("modbus_stop", 1)

    def _modbus_block_a_settings():
        entries = getattr(app, "modbus_block_a_entries", {})
        return {
            "ip": (entries["ip"].get().strip() or "MOCK") if "ip" in entries else "MOCK",
            "port": int(_entry_value(entries.get("port"), 502, int)) if "port" in entries else 502,
            "slave_id": int(_entry_value(entries.get("slave_id"), 1, int)) if "slave_id" in entries else 1,
            "timeout_ms": int(_entry_value(entries.get("timeout_ms"), 1000, int)) if "timeout_ms" in entries else 1000,
            "iterations": int(_entry_value(entries.get("iterations"), 1000, int)) if "iterations" in entries else 1000,
            "function": app.modbus_block_a_function.get() if hasattr(app, "modbus_block_a_function") else "read_holding_register",
            "address": int(_entry_value(entries.get("address"), 100, int)) if "address" in entries else 100,
            "count": int(_entry_value(entries.get("count"), 1, int)) if "count" in entries else 1,
        }

    def _modbus_block_b_settings():
        entries = getattr(app, "modbus_block_b_entries", {})
        return {
            "iterations": int(_entry_value(entries.get("iterations"), 100, int)) if "iterations" in entries else 100,
            "trigger_name": (entries["trigger_name"].get().strip() or "trigger_pick") if "trigger_name" in entries else "trigger_pick",
            "done_name": (entries["done_name"].get().strip() or "cycle_done") if "done_name" in entries else "cycle_done",
            "error_name": (entries["error_name"].get().strip() or "error_flag") if "error_name" in entries else "error_flag",
        }

    def _save_modbus_block_settings():
        block_a = _modbus_block_a_settings() if hasattr(app, "modbus_block_a_entries") else None
        block_b = _modbus_block_b_settings() if hasattr(app, "modbus_block_b_entries") else None
        modbus_manager.save_block_test_config(block_a=block_a, block_b=block_b)
        return block_a, block_b

    def start_modbus_block_a():
        block_a, _ = _save_modbus_block_settings()
        if block_a is not None:
            modbus_manager.start_block_a(block_a)
            research_logger.record("modbus_block_a_ui_start", int(block_a.get("iterations", 0)))

    def stop_modbus_block_a():
        modbus_manager.stop_block_a()
        research_logger.record("modbus_block_a_ui_stop", 1)

    def start_modbus_block_b():
        save_modbus_settings()
        modbus_manager.start()
        _, block_b = _save_modbus_block_settings()
        if block_b is not None:
            app._modbus_block_b_cycle_active = False
            modbus_manager.start_block_b(block_b)
            research_logger.record("modbus_block_b_ui_start", int(block_b.get("iterations", 0)))

    def stop_modbus_block_b():
        app._modbus_block_b_cycle_active = False
        modbus_manager.stop_block_b()
        research_logger.record("modbus_block_b_ui_stop", 1)

    def set_research_enabled(enabled):
        research_logger.set_enabled(enabled)
        if enabled:
            app._research_record_start = time.monotonic()
        state = "started" if enabled else "stopped"
        shared_string.value = f"Log: Research recording {state}".encode("utf-8")[:99]

    def reset_research():
        research_logger.reset()
        clear_timestamp_table()
        app._timestamp_table_signature = None
        if hasattr(app, "timestamp_tree"):
            app.timestamp_tree.delete(*app.timestamp_tree.get_children())
        app._research_record_start = 0.0
        research_logger.record("research_reset", 1)

    def export_research_csv():
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile=research_logger.path.name,
            filetypes=(("CSV Files", "*.csv"), ("Excel Files", "*.xlsx")),
        )
        if path:
            if os.path.splitext(path)[1].lower() == ".xlsx":
                exported = research_logger.export_excel(path)
            else:
                exported = research_logger.export_csv(path)
            research_logger.record("research_export", 1, str(exported))

    def reset_timestamp_table():
        clear_timestamp_table()
        app._timestamp_table_signature = None
        if hasattr(app, "timestamp_tree"):
            app.timestamp_tree.delete(*app.timestamp_tree.get_children())
        research_logger.record("timestamp_table_reset", 1)

    def export_timestamp_table():
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile=f"timestamp_table_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            filetypes=(("CSV Files", "*.csv"), ("Excel Files", "*.xlsx")),
        )
        if not path:
            return
        try:
            exported = export_timestamp_table_file(path)
        except Exception as exc:
            messagebox.showerror("Timestamp Export", str(exc))
            return
        research_logger.record("timestamp_table_export", 1, str(exported))
        messagebox.showinfo("Timestamp Export", "Exported: " + str(exported))

    def _update_vision_ui(vision_state):
        if not hasattr(app, "vision_status"):
            return
        latest = vision_state.get("latest") if isinstance(vision_state, dict) else None
        bundle = vision_state.get("latest_bundle") if isinstance(vision_state, dict) else None
        status = str(vision_state.get("status", "stopped")) if isinstance(vision_state, dict) else "stopped"
        app.vision_status.configure(text="Status: " + status)
        app.vision_model_status.configure(text="Model: " + str(vision_state.get("model_status", "NOT LOADED")))
        safety = "UNKNOWN"
        if isinstance(bundle, dict) and bundle.get("pick_safety"):
            safety = str(bundle.get("pick_safety")).upper()
        elif isinstance(latest, dict) and latest.get("workspace_status"):
            safety = str(latest.get("workspace_status")).upper()
        color_map = {"SAFE": "#7AC922", "VALID": "#7AC922", "MARGINAL": "#F5A623", "MARGIN": "#F5A623", "UNSAFE": "#E84040", "OUT": "#E84040", "UNKNOWN": "#8A9AB8"}
        app.vision_safety_badge.configure(text=safety, text_color=color_map.get(safety, "#8A9AB8"))
        if isinstance(latest, dict):
            app.vision_info_labels["Status"].configure(text=str(latest.get("workspace_message", latest.get("status", "-"))))
            app.vision_info_labels["BBox"].configure(text=str(latest.get("bbox_px", "-")))
            app.vision_info_labels["Centroid"].configure(text=f"px={latest.get('centroid_px', '-')} | world={latest.get('centroid_world', '-')}")
            app.vision_info_labels["Dimension"].configure(text=f"W={latest.get('width_mm', 0)} mm | H={latest.get('height_mm', 0)} mm")
            app.vision_info_labels["Pick point"].configure(text=str(latest.get("pick_point_world", "-")))
            app.vision_info_labels["Orientation"].configure(text=str(latest.get("orientation_deg", "-")))
        else:
            for label in app.vision_info_labels.values():
                label.configure(text="-")
        calibration = vision_state.get("calibration", {}) if isinstance(vision_state, dict) else {}
        if isinstance(calibration, dict):
            app.vision_calibration.configure(text=f"Calibration: fx={calibration.get('fx', 0)} fy={calibration.get('fy', 0)} snapshots={calibration.get('snapshots_captured', 0)}")
        confirm = read_state("vision_confirmation", {})
        if isinstance(confirm, dict) and confirm.get("status") == "pending":
            app.vision_marginal_frame.grid()
            try:
                token = float(confirm.get("token", 0))
                timeout_s = float(load_config().get("vision", {}).get("marginal_confirm_timeout_s", 2.0))
                remaining = max(0.0, token + timeout_s - time.time())
                app.vision_marginal_countdown.configure(text=f"Auto-skip in {remaining:.1f}s")
            except Exception:
                app.vision_marginal_countdown.configure(text="")
        else:
            app.vision_marginal_frame.grid_remove()
            app.vision_marginal_countdown.configure(text="")
        ibvs_state = read_state("vision_ibvs", {})
        if hasattr(app, "vision_ibvs_status") and isinstance(ibvs_state, dict):
            if ibvs_state.get("active"):
                err = ibvs_state.get("last_err_px", {})
                cmd = ibvs_state.get("last_cmd_mm", {})
                app.vision_ibvs_status.configure(
                    text=(
                        f"IBVS: {ibvs_state.get('status', 'tracking')} | "
                        f"err=({err.get('u_err', 0):.1f},{err.get('v_err', 0):.1f}) px | "
                        f"cmd=({cmd.get('dx', 0):.2f},{cmd.get('dy', 0):.2f}) mm"
                    )
                )
            else:
                app.vision_ibvs_status.configure(text="IBVS: idle")
        if getattr(app, "current_menu", "") == "Vision":
            now = time.monotonic()
            if now - getattr(app, "_last_vision_image_update", 0) > 0.12:
                image = vision_manager.latest_frame_image()
                if image is not None:
                    app._vision_ctk_image = customtkinter.CTkImage(light_image=image, dark_image=image, size=image.size)
                    app.vision_feed_label.configure(image=app._vision_ctk_image, text="")
                app._last_vision_image_update = now

    def _set_tree_rows(tree, rows):
        for item in tree.get_children():
            tree.delete(item)
        for values in rows:
            tree.insert("", "end", values=values)

    def release_modbus_jog(button_index):
        try:
            Joint_jog_buttons[button_index] = 0
        except Exception:
            pass

    def _handle_modbus_jog(snapshot):
        trigger = bool(snapshot.get("jog_trigger", False))
        if not hasattr(app, "_last_modbus_jog_trigger"):
            app._last_modbus_jog_trigger = False
        rising = trigger and not app._last_modbus_jog_trigger
        app._last_modbus_jog_trigger = trigger
        if not rising or not bool(snapshot.get("jog_enable", False)):
            return
        joint_id = int(bool(snapshot.get("jog_joint_bit0", False))) | (int(bool(snapshot.get("jog_joint_bit1", False))) << 1) | (int(bool(snapshot.get("jog_joint_bit2", False))) << 2)
        if joint_id < 0 or joint_id > 5:
            return
        cfg = load_config()["modbus"]
        now = time.monotonic()
        timeout = float(cfg.get("jog_lock_timeout_s", 5.0))
        owner = getattr(app, "_jog_owner", None)
        expiry = getattr(app, "_jog_owner_expiry", 0.0)
        if owner not in (None, "modbus") and now < expiry:
            research_logger.record("modbus_jog_blocked", 1, str(owner))
            return
        app._jog_owner = "modbus"
        app._jog_owner_expiry = now + timeout
        Jog_control[0] = int(float(cfg.get("jog_speed_pct", 20)))
        direction_positive = bool(snapshot.get("jog_direction", False))
        button_index = joint_id if direction_positive else joint_id + 6
        Joint_jog_buttons[button_index] = 1
        app.modbus_jog_source.configure(text="Active jog source: MODBUS")
        research_logger.record("modbus_jog", joint_id + 1, "positive" if direction_positive else "negative")
        app.after(220, lambda idx=button_index: release_modbus_jog(idx))

    def _update_modbus_block_a_ui():
        if not hasattr(app, "modbus_block_a_labels"):
            return
        state = read_state("modbus_block_a", {})
        if not isinstance(state, dict):
            state = {}
        stats = state.get("stats", {}) if isinstance(state.get("stats"), dict) else {}
        completed = int(state.get("completed", 0) or 0)
        target = int(state.get("target", 0) or 0)
        app.modbus_block_a_labels["progress"].configure(text=f"progress: {completed}/{target} {'RUNNING' if state.get('running') else 'IDLE'}")
        for key in ["avg_rt_ms", "throughput_rps", "packet_loss_pct", "success_count", "failed_count", "timeout_count"]:
            app.modbus_block_a_labels[key].configure(text=f"{key}: {stats.get(key, '-')}")

    def _handle_modbus_block_b(snapshot):
        state = read_state("modbus_block_b", {})
        if not isinstance(state, dict) or not state.get("running"):
            return False
        trigger_name = str(state.get("trigger_name", "trigger_pick"))
        trigger = bool(snapshot.get(trigger_name, False))
        if not hasattr(app, "_last_modbus_block_b_trigger"):
            app._last_modbus_block_b_trigger = False
        rising = trigger and not app._last_modbus_block_b_trigger
        app._last_modbus_block_b_trigger = trigger

        if rising and not state.get("active_cycle") and Buttons[7] == 0:
            if modbus_manager.block_b_cycle_started():
                app._modbus_block_b_cycle_active = True
                execute_program()
                return True

        if getattr(app, "_modbus_block_b_cycle_active", False):
            control = read_state("program_control", {})
            control = control if isinstance(control, dict) else {}
            program_state = str(control.get("state", "")).upper()
            if Buttons[7] == 0 and program_state in {"IDLE", "ERROR", "STOP_REQUESTED", "RUNNING"}:
                success = program_state == "IDLE" and not bool(control.get("stop_requested", False))
                modbus_manager.block_b_cycle_finished(success, program_state)
                app._modbus_block_b_cycle_active = False
        return True

    def _update_modbus_block_b_ui():
        if not hasattr(app, "modbus_block_b_labels"):
            return
        state = read_state("modbus_block_b", {})
        if not isinstance(state, dict):
            state = {}
        stats = state.get("stats", {}) if isinstance(state.get("stats"), dict) else {}
        completed = int(state.get("completed", 0) or 0)
        target = int(state.get("target", 0) or 0)
        app.modbus_block_b_labels["progress"].configure(text=f"progress: {completed}/{target} {'RUNNING' if state.get('running') else 'IDLE'}")
        app.modbus_block_b_labels["active"].configure(text=f"active: {bool(state.get('active_cycle', False))}")
        app.modbus_block_b_labels["success_count"].configure(text=f"success_count: {state.get('success_count', 0)}")
        app.modbus_block_b_labels["failed_count"].configure(text=f"failed_count: {state.get('failed_count', 0)}")
        for key in ["avg_s", "min_s", "max_s", "std_s"]:
            app.modbus_block_b_labels[key].configure(text=f"{key}: {stats.get(key, '-')}")

    def _update_modbus_ui(modbus_state):
        if not hasattr(app, "modbus_status"):
            return
        if not isinstance(modbus_state, dict):
            modbus_state = {}
        snapshot = modbus_state.get("snapshot", {}) if isinstance(modbus_state, dict) else {}
        app.modbus_status.configure(text="Status: " + str(modbus_state.get("description", "Disconnected")))
        _set_tree_rows(app.modbus_monitor_table, [(key, value) for key, value in snapshot.items()])
        _update_modbus_block_a_ui()
        block_b_running = _handle_modbus_block_b(snapshot)
        _update_modbus_block_b_ui()
        trigger = bool(snapshot.get("trigger_pick", False))
        if not hasattr(app, "_last_modbus_trigger"):
            app._last_modbus_trigger = False
        if not block_b_running and trigger and not app._last_modbus_trigger and Buttons[7] == 0:
            research_logger.record("modbus_trigger_pick", 1)
            execute_program()
        app._last_modbus_trigger = trigger
        _handle_modbus_jog(snapshot)
        if getattr(app, "_jog_owner", None) == "modbus" and time.monotonic() > getattr(app, "_jog_owner_expiry", 0.0):
            app._jog_owner = None
            app.modbus_jog_source.configure(text="Active jog source: NONE")

    def _timestamp_cell(value):
        if value is None:
            return ""
        if isinstance(value, float):
            return f"{value:.3f}" if value else ""
        return str(value)

    def _update_timestamp_table_ui(timestamp_state):
        if not hasattr(app, "timestamp_tree"):
            return
        timestamp_state = timestamp_state if isinstance(timestamp_state, dict) else {}
        rows = timestamp_state.get("rows", [])
        rows = rows if isinstance(rows, list) else []
        signature = (len(rows), str(timestamp_state.get("updated_at", "")), str(timestamp_state.get("last_row", "")))
        if signature == getattr(app, "_timestamp_table_signature", None):
            return
        app.timestamp_tree.delete(*app.timestamp_tree.get_children())
        for index, row in enumerate(rows[-300:]):
            if not isinstance(row, dict):
                continue
            app.timestamp_tree.insert(
                "",
                "end",
                iid=f"timestamp_{index}",
                values=(
                    row.get("time", ""),
                    row.get("event", ""),
                    row.get("label", ""),
                    _timestamp_cell(row.get("elapsed_s", "")),
                    _timestamp_cell(row.get("duration_s", "")),
                ),
            )
        children = app.timestamp_tree.get_children()
        if children:
            app.timestamp_tree.see(children[-1])
        app._timestamp_table_signature = signature

    def _update_research_ui(research_state):
        if not hasattr(app, "research_last"):
            return
        enabled = bool(research_state.get("enabled", False)) if isinstance(research_state, dict) else False
        app.research_status.configure(text="RECORDING" if enabled else "IDLE")
        if enabled and getattr(app, "_research_record_start", 0.0):
            elapsed = int(time.monotonic() - app._research_record_start)
            app.research_elapsed.configure(text=f"{elapsed // 60:02d}:{elapsed % 60:02d}")
        app.research_path.configure(text="File: " + str(research_state.get("path", research_logger.path)))
        last_event = research_state.get("last_event", "none") if isinstance(research_state, dict) else "none"
        app.research_last.configure(text="Last event: " + str(last_event))
        summary = research_state.get("summary", {}) if isinstance(research_state, dict) else {}
        for key, label in app.research_kpi_labels.items():
            label.configure(text=str(summary.get(key, "--")))
        recent = research_state.get("recent", []) if isinstance(research_state, dict) else []
        last_count = getattr(app, "_research_recent_count", -1)
        if len(recent) != last_count:
            app.research_events.delete("1.0", tk.END)
            for row in recent[-80:]:
                app.research_events.insert(tk.END, f"[{row.get('timestamp','')}] {row.get('metric','')}={row.get('value','')} {row.get('note','')}\n")
            app.research_events.see(tk.END)
            app._research_recent_count = len(recent)
        _update_timestamp_table_ui(read_state("research_timestamp_table", {}))
        now = time.monotonic()
        if now - getattr(app, "_last_research_plot_update", 0.0) > 0.7:
            series = research_logger.metric_series(["serial_latency", "modbus_cycle", "vision_time", "cycle_time", "timestamp_duration_s", "modbus_block_b_cycle_time_s"])
            for key, axis in app.research_axes.items():
                axis.clear()
                axis.set_facecolor(UI_SURFACE_LOW)
                axis.set_title(key.replace("_", " ").title(), color=UI_ON_SURFACE, fontsize=8, pad=4)
                axis.grid(True, alpha=0.25, color=UI_ON_SURFACE_MUTE)
                axis.tick_params(axis='both', which='major', labelsize=7, colors=UI_ON_SURFACE_MUTE)
                for spine in ['top', 'bottom', 'left', 'right']:
                    axis.spines[spine].set_color(UI_BORDER)
                values = series.get(key, [])
                if values:
                    axis.plot(values, color=UI_ACCENT, linewidth=1.5)
            app.research_plot_fig.tight_layout()
            app.research_plot_canvas.draw_idle()
            app._last_research_plot_update = now


    def demo_start():
        Buttons[6] = 1
        None

    def demo_stop():
        Buttons[6] = 0
        None


    def translation_press(event=None,var=0, var2 = 0):
        translation_buttons[var2] = var
        Cart_jog_buttons[var2] = var
        logging.debug(translation_buttons)
        logging.debug("CART JOG PRESS " + str(list(Cart_jog_buttons)))

    def translation_release(event=None,var=0, var2 = 0):
        translation_buttons[var2] = var
        Cart_jog_buttons[var2] = var
        logging.debug(translation_buttons)
        logging.debug("CART JOG RELEASE " + str(list(Cart_jog_buttons)))

    def make_lambda_press(x,var2):
        return lambda ev:translation_press(ev,x,var2)        

    def make_lambda_release(x,var2):
        return lambda ev:translation_release(ev,x,var2)   

    def rotation_press(event=None,var=0, var2 = 0):
        rotation_buttons[var2] = var
        Cart_jog_buttons[var2+6] = var
        logging.debug(rotation_buttons)
        logging.debug("CART JOG PRESS " + str(list(Cart_jog_buttons)))


    def rotation_release(event=None,var=0, var2 = 0):
        rotation_buttons[var2] = var
        Cart_jog_buttons[var2+6] = var
        logging.debug(rotation_buttons)
        logging.debug("CART JOG RELEASE " + str(list(Cart_jog_buttons)))

    def make_lambda_press_rot(x,var2):
        return lambda ev:rotation_press(ev,x,var2)        

    def make_lambda_release_rot(x,var2):
        return lambda ev:rotation_release(ev,x,var2)   


    def cart_jog_frame():


        app.WRF_select = customtkinter.CTkRadioButton(master=app.cart_frame.content_frame, text="WRF",  value=0,command = WRF_button)
        app.WRF_select.grid(row=0, column=2, pady=10, padx=20, sticky="we")

        app.TRF_select = customtkinter.CTkRadioButton(master=app.cart_frame.content_frame, text="TRF",  value=2,command = TRF_button)
        app.TRF_select.grid(row=0, column=3, pady=10, padx=20, sticky="we")

        app.joint_jog = customtkinter.CTkButton(app.cart_frame.content_frame,text="Joint jog", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command = raise_frame_jog)
        app.joint_jog.grid(row=0, column=0, padx=20,pady = (10,20),sticky="news")

        app.cart_jog = customtkinter.CTkButton(app.cart_frame.content_frame,text="Cartesian jog", font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command = raise_frame_cart)
        app.cart_jog.grid(row=0, column=1, padx=20,pady = (10,20),sticky="news")

        app.TRF_select.select()
        #cart z up and down            
        z_up =Image.open(os.path.join(Image_path, "cart_z_up.png"))
        z_down =Image.open(os.path.join(Image_path, "cart_z_down.png")) #z_up.rotate(180)

        app.z_up = customtkinter.CTkImage(z_up, size=(80, 80))
        app.z_up_button = customtkinter.CTkButton(app.cart_frame.content_frame, corner_radius=0, height=10, border_spacing=10,
                                                    fg_color="transparent", text_color=("gray10", "gray90"),
                                                    image=app.z_up, anchor="CENTER",text = "",hover = 0) #hover = 0
        app.z_up_button.place(x = 530, y = 80)
        app.z_up_button.bind('<ButtonPress-1>',make_lambda_press(1,4))
        app.z_up_button.bind('<ButtonRelease-1>',make_lambda_release(0,4))

        app.z_down = customtkinter.CTkImage(z_down, size=(80, 80))
        app.z_down_button = customtkinter.CTkButton(app.cart_frame.content_frame, corner_radius=0, height=10, border_spacing=10,
                                                    fg_color="transparent", text_color=("gray10", "gray90"),
                                                    image=app.z_down, anchor="CENTER",text = "",hover = 0) #hover = 0
        app.z_down_button.place(x = 530, y = 260)
        app.z_down_button.bind('<ButtonPress-1>',make_lambda_press(1,5))
        app.z_down_button.bind('<ButtonRelease-1>',make_lambda_release(0,5))


        #cart x up
        x_up =Image.open(os.path.join(Image_path, "cart_x_up.png"))
        app.x_up = customtkinter.CTkImage(x_up, size=(80, 50))
        app.x_up_button = customtkinter.CTkButton(app.cart_frame.content_frame, corner_radius=0, height=10, border_spacing=10,
                                                    fg_color="transparent", text_color=("gray10", "gray90"),
                                                    image=app.x_up, anchor="CENTER",text = "",hover = 0) #hover = 0
        app.x_up_button.place(x = 170, y = 60)
        app.x_up_button.bind('<ButtonPress-1>',make_lambda_press(1,0))
        app.x_up_button.bind('<ButtonRelease-1>',make_lambda_release(0,0))

        #cart x down
        x_down =Image.open(os.path.join(Image_path, "cart_x_down.png"))
        app.x_down = customtkinter.CTkImage(x_down, size=(120, 120))
        app.x_down_button = customtkinter.CTkButton(app.cart_frame.content_frame, corner_radius=0, height=10, border_spacing=10,
                                                    fg_color="transparent", text_color=("gray10", "gray90"),
                                                    image=app.x_down, anchor="CENTER",text = "",hover = 0) #hover = 0
        app.x_down_button.place(x = 160, y = 260)
        app.x_down_button.bind('<ButtonPress-1>',make_lambda_press(1,1))
        app.x_down_button.bind('<ButtonRelease-1>',make_lambda_release(0,1))

        #cart y left right
        y_left =Image.open(os.path.join(Image_path, "cart_y_left.png"))
        y_right =Image.open(os.path.join(Image_path, "cart_y_right.png")) # y_left.transpose(PIL.Image.Transpose.FLIP_LEFT_RIGHT)  

        app.y_left = customtkinter.CTkImage(y_left, size=(90, 90))
        app.y_left_button = customtkinter.CTkButton(app.cart_frame.content_frame, corner_radius=0, height=10, border_spacing=10,
                                                    fg_color="transparent", text_color=("gray10", "gray90"),
                                                    image=app.y_left, anchor="CENTER",text = "",hover = 0) #hover = 0
        app.y_left_button.place(x = 40, y = 150)
        app.y_left_button.bind('<ButtonPress-1>',make_lambda_press(1,2))
        app.y_left_button.bind('<ButtonRelease-1>',make_lambda_release(0,2))



        app.y_right = customtkinter.CTkImage(y_right, size=(90, 90))
        app.y_right_button = customtkinter.CTkButton(app.cart_frame.content_frame, corner_radius=0, height=10, border_spacing=10,
                                                    fg_color="transparent", text_color=("gray10", "gray90"),
                                                    image=app.y_right, anchor="CENTER",text = "",hover = 0) #hover = 0
        app.y_right_button.place(x = 300, y = 150)
        app.y_right_button.bind('<ButtonPress-1>',make_lambda_press(1,3))
        app.y_right_button.bind('<ButtonRelease-1>',make_lambda_release(0,3))


        #cart rot x + x-
        rot_x_pos =Image.open(os.path.join(Image_path, "RX_MINUS.png"))
        app.rot_x_pos = customtkinter.CTkImage(rot_x_pos, size=(90, 90))
        app.rot_x_pos_button = customtkinter.CTkButton(app.cart_frame.content_frame, corner_radius=0, height=10, border_spacing=10,
                                                    fg_color="transparent", text_color=("gray10", "gray90"),
                                                    image=app.rot_x_pos, anchor="CENTER",text = "",hover = 0) #hover = 0
        app.rot_x_pos_button.place(x = 400, y = 490)
        app.rot_x_pos_button.bind('<ButtonPress-1>',make_lambda_press_rot(1,0))
        app.rot_x_pos_button.bind('<ButtonRelease-1>',make_lambda_release_rot(0,0))

        rot_x_neg =Image.open(os.path.join(Image_path, "RX_PLUS.png"))
        app.rot_x_neg = customtkinter.CTkImage(rot_x_neg, size=(90, 90))
        app.rot_x_neg_button = customtkinter.CTkButton(app.cart_frame.content_frame, corner_radius=0, height=10, border_spacing=10,
                                                    fg_color="transparent", text_color=("gray10", "gray90"),
                                                    image=app.rot_x_neg, anchor="CENTER",text = "",hover = 0) #hover = 0
        app.rot_x_neg_button.place(x = 400, y = 330)
        app.rot_x_neg_button.bind('<ButtonPress-1>',make_lambda_press_rot(1,1))
        app.rot_x_neg_button.bind('<ButtonRelease-1>',make_lambda_release_rot(0,1))


        #cart rot y + y-
        rot_y_pos =Image.open(os.path.join(Image_path, "RY_PLUS.png"))
        app.rot_y_pos = customtkinter.CTkImage(rot_y_pos, size=(90, 90))
        app.rot_y_pos_button = customtkinter.CTkButton(app.cart_frame.content_frame, corner_radius=0, height=10, border_spacing=10,
                                                    fg_color="transparent", text_color=("gray10", "gray90"),
                                                    image=app.rot_y_pos, anchor="CENTER",text = "",hover = 0) #hover = 0
        app.rot_y_pos_button.place(x = 300, y = 410)
        app.rot_y_pos_button.bind('<ButtonPress-1>',make_lambda_press_rot(1,2))
        app.rot_y_pos_button.bind('<ButtonRelease-1>',make_lambda_release_rot(0,2))


        rot_y_neg=Image.open(os.path.join(Image_path, "RY_MINUS.png"))
        app.rot_y_neg = customtkinter.CTkImage(rot_y_neg, size=(90, 90))
        app.rot_y_neg_button = customtkinter.CTkButton(app.cart_frame.content_frame, corner_radius=0, height=10, border_spacing=10,
                                                    fg_color="transparent", text_color=("gray10", "gray90"),
                                                    image=app.rot_y_neg, anchor="CENTER",text = "",hover = 0) #hover = 0
        app.rot_y_neg_button.place(x = 500, y = 410)
        app.rot_y_neg_button.bind('<ButtonPress-1>',make_lambda_press_rot(1,3))
        app.rot_y_neg_button.bind('<ButtonRelease-1>',make_lambda_release_rot(0,3))


        #cart rot z + z -
        rot_z_pos_rot =Image.open(os.path.join(Image_path, "RZ_MINUS.png"))
        #rot_z_pos_rot = rot_z_pos.rotate(270)
        app.rot_z_pos = customtkinter.CTkImage(rot_z_pos_rot, size=(90, 90))
        app.rot_z_pos_button = customtkinter.CTkButton(app.cart_frame.content_frame, corner_radius=0, height=10, border_spacing=10,
                                                    fg_color="transparent", text_color=("gray10", "gray90"),
                                                    image=app.rot_z_pos, anchor="CENTER",text = "",hover = 0) #hover = 0
        app.rot_z_pos_button.place(x = 50, y = 490)
        app.rot_z_pos_button.bind('<ButtonPress-1>',make_lambda_press_rot(1,5))
        app.rot_z_pos_button.bind('<ButtonRelease-1>',make_lambda_release_rot(0,5))

        rot_z_neg =Image.open(os.path.join(Image_path, "RZ_PLUS.png"))
        app.rot_z_neg = customtkinter.CTkImage(rot_z_neg, size=(90, 90))
        app.rot_z_neg_button = customtkinter.CTkButton(app.cart_frame.content_frame, corner_radius=0, height=10, border_spacing=10,
                                                    fg_color="transparent", text_color=("gray10", "gray90"),
                                                    image=app.rot_z_neg, anchor="CENTER",text = "",hover = 0) #hover = 0
        app.rot_z_neg_button.place(x = 50, y = 370)
        app.rot_z_neg_button.bind('<ButtonPress-1>',make_lambda_press_rot(1,4))
        app.rot_z_neg_button.bind('<ButtonRelease-1>',make_lambda_release_rot(0,4))

    def robot_positions_frames():
        #robot positions frame
        app.joint_positions_frame = CollapsibleFrame(app.left_scrollable_container, title="Robot Positions & Speed Controls")
        app.joint_positions_frame.grid(row=1, column=0, columnspan=1, padx=(5,0), pady=5, sticky="news")
        app.joint_positions_frame.content_frame.grid_columnconfigure(0, weight=1)
        app.joint_positions_frame.content_frame.grid_columnconfigure(1, weight=1)
        app.joint_positions_frame.content_frame.grid_columnconfigure(2, weight=0)
        app.joint_positions_frame.content_frame.grid_columnconfigure(3, weight=1)
        app.joint_positions_frame.content_frame.grid_columnconfigure(4, weight=2)
        app.joint_positions_frame.content_frame.grid_columnconfigure(5, weight=1)
        app.joint_positions_frame.content_frame.grid_rowconfigure(0, weight=0)

        # Tool positions
        app.tools_positions = customtkinter.CTkLabel(app.joint_positions_frame.content_frame, text="Tools positions:", font=customtkinter.CTkFont(size=14, weight="bold"))
        app.tools_positions.grid(row=0, column=0, padx=6, pady=3, sticky="w")

        app.x_pos = customtkinter.CTkLabel(app.joint_positions_frame.content_frame, text="X: "+ str(1929).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.x_pos.grid(row=1, column=0, padx=6, pady=2, sticky="w")

        app.y_pos = customtkinter.CTkLabel(app.joint_positions_frame.content_frame, text="Y: "+ str(1929).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.y_pos.grid(row=2, column=0, padx=6, pady=2, sticky="w")

        app.z_pos = customtkinter.CTkLabel(app.joint_positions_frame.content_frame, text="Z: "+ str(1929).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.z_pos.grid(row=3, column=0, padx=6, pady=2, sticky="w")

        app.Rx_pos = customtkinter.CTkLabel(app.joint_positions_frame.content_frame, text="Rx: "+ str(1929).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.Rx_pos.grid(row=4, column=0, padx=6, pady=2, sticky="w")

        app.Ry_pos = customtkinter.CTkLabel(app.joint_positions_frame.content_frame, text="Ry: "+ str(1929).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.Ry_pos.grid(row=5, column=0, padx=6, pady=2, sticky="w")

        app.Rz_pos = customtkinter.CTkLabel(app.joint_positions_frame.content_frame, text="Rz: "+ str(1929).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.Rz_pos.grid(row=6, column=0, padx=6, pady=2, sticky="w")

        # Joint positions
        app.joint_positions = customtkinter.CTkLabel(app.joint_positions_frame.content_frame, text="Joint positions:", font=customtkinter.CTkFont(size=14, weight="bold"))
        app.joint_positions.grid(row=0, column=1, padx=6, pady=3, sticky="w")

        app.theta1 = customtkinter.CTkLabel(app.joint_positions_frame.content_frame, text="θ1: " + str(1929).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.theta1.grid(row=1, column=1, padx=6, pady=2, sticky="w")

        app.theta2 = customtkinter.CTkLabel(app.joint_positions_frame.content_frame, text="θ2: " + str(1929).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.theta2.grid(row=2, column=1, padx=6, pady=2, sticky="w")

        app.theta3 = customtkinter.CTkLabel(app.joint_positions_frame.content_frame, text="θ3: " + str(1929).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.theta3.grid(row=3, column=1, padx=6, pady=2, sticky="w")

        app.theta4 = customtkinter.CTkLabel(app.joint_positions_frame.content_frame, text="θ4: " + str(199).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.theta4.grid(row=4, column=1, padx=6, pady=2, sticky="w")

        app.theta5 = customtkinter.CTkLabel(app.joint_positions_frame.content_frame, text="θ5: " + str(1929).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.theta5.grid(row=5, column=1, padx=6, pady=2, sticky="w")

        app.theta6 = customtkinter.CTkLabel(app.joint_positions_frame.content_frame, text="θ6: " + str(1929).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.theta6.grid(row=6, column=1, padx=6, pady=2, sticky="w")

        # Column spacer / labels
        app.dummy_label2 = customtkinter.CTkLabel(app.joint_positions_frame.content_frame,text="", font = customtkinter.CTkFont(size=12, family='TkDefaultFont'))
        app.dummy_label3 = customtkinter.CTkLabel(app.joint_positions_frame.content_frame,text="", font = customtkinter.CTkFont(size=12, family='TkDefaultFont'))
        app.dummy_label2.grid(row=0, column=4, padx=5, pady=3, sticky="news")
        app.dummy_label3.grid(row=0, column=5, padx=5, pady=3, sticky="news")

        app.Velocity_label = customtkinter.CTkLabel(app.joint_positions_frame.content_frame,text="JOG velocity", font = customtkinter.CTkFont(size=13, family='TkDefaultFont'))
        app.Velocity_label.grid(row=1, column=3, padx=6, pady=3, sticky="w")

        app.slider1 = customtkinter.CTkSlider(app.joint_positions_frame.content_frame,from_ = 0, to = 100,number_of_steps=100)
        app.slider1.set(50)
        app.slider1.grid(row=1, column=4, columnspan=1, padx=(6, 6), pady=3, sticky="ew")

        app.Velocity_percent = customtkinter.CTkLabel(app.joint_positions_frame.content_frame,text="100%", font = customtkinter.CTkFont(size=14, family='TkDefaultFont'))
        app.Velocity_percent.grid(row=1, column=5, padx=4, pady=3, sticky="w")

        app.Accel_label = customtkinter.CTkLabel(app.joint_positions_frame.content_frame,text="JOG accel", font = customtkinter.CTkFont(size=13, family='TkDefaultFont'))
        app.Accel_label.grid(row=2, column=3, padx=6, pady=3, sticky="w")

        app.slider2 = customtkinter.CTkSlider(app.joint_positions_frame.content_frame,from_ = 0, to = 100,number_of_steps=100)
        app.slider2.set(50)
        app.slider2.grid(row=2, column=4, columnspan=1, padx=(6, 6), pady=3, sticky="ew")

        app.Accel_percent = customtkinter.CTkLabel(app.joint_positions_frame.content_frame,text="100%", font = customtkinter.CTkFont(size=14, family='TkDefaultFont'))
        app.Accel_percent.grid(row=2, column=5, padx=4, pady=3, sticky="w")

        app.Incremental_jog = customtkinter.CTkLabel(app.joint_positions_frame.content_frame,text="Incremental", font = customtkinter.CTkFont(size=13, family='TkDefaultFont'))
        app.Incremental_jog.grid(row=3, column=3, padx=6, pady=3, sticky="w")

        app.Incremental_jog_button = customtkinter.CTkRadioButton(master=app.joint_positions_frame.content_frame, text="", value=2)
        app.Incremental_jog_button.grid(row=3, column=4, pady=3, padx=6, sticky="w")

        app.Incremental_jog_step = customtkinter.CTkLabel(app.joint_positions_frame.content_frame,text="Step size", font = customtkinter.CTkFont(size=13, family='TkDefaultFont'))
        app.Incremental_jog_step.grid(row=4, column=3, padx=6, pady=3, sticky="w")

        app.Step_entry = customtkinter.CTkEntry(app.joint_positions_frame.content_frame, width=50)
        app.Step_entry.grid(row=4, column=4, padx=6, pady=3, sticky="w")

        app.enable_disable = customtkinter.CTkButton(app.joint_positions_frame.content_frame,text="Enable", font = customtkinter.CTkFont(size=14, family='TkDefaultFont'),command = Enable_press)
        app.enable_disable.grid(row=5, column=3, padx=4, pady=4, sticky="we")

        app.enable_disable_2 = customtkinter.CTkButton(app.joint_positions_frame.content_frame,text="Disable", font = customtkinter.CTkFont(size=14, family='TkDefaultFont'),command = Disable_press)
        app.enable_disable_2.grid(row=6, column=3, padx=4, pady=4, sticky="we")

        app.Quick_gripper_on_off = customtkinter.CTkRadioButton(master=app.joint_positions_frame.content_frame, text="Gripper", command = quick_gripper_button )
        app.Quick_gripper_on_off.grid(row=7, column=4, columnspan=2, pady=4, padx=6, sticky="w")

        app.home = customtkinter.CTkButton(app.joint_positions_frame.content_frame,text="Home", font = customtkinter.CTkFont(size=14, family='TkDefaultFont'),command = Home_robot)
        app.home.grid(row=5, column=4, padx=4, pady=4, sticky="we")

        app.park = customtkinter.CTkButton(app.joint_positions_frame.content_frame,text="Park", font = customtkinter.CTkFont(size=14, family='TkDefaultFont'),command = Park_robot)
        app.park.grid(row=6, column=4, padx=4, pady=4, sticky="we") 


        
    def highlight_words_program(event):
        app.textbox_program.tag_config("green", foreground="green")
        words = PAROL6_ROBOT.Commands_list

        for word in words:
            start = "1.0"
            while True:
                start = app.textbox_program.search(word, start, stopindex=tk.END)
                if not start:
                    break
                end = f"{start}+{len(word)}c"
                app.textbox_program.tag_add("green", start, end)
                start = end

    def highlight_words_response(event):
        app.textbox_response.tag_config("red", foreground="red")
        words = ["Warrning", "Error", "Log"]
        for word in words:
            start = "1.0"
            while True:
                start = app.textbox_response.search(word, start, stopindex=tk.END)
                if not start:
                    break
                end = f"{start}+{len(word)}c"
                app.textbox_response.tag_add("red", start, end)
                start = end


    # dodaj slikice kao iz meca studio
    def program_frames():
        #program frame
        app.program_frame = customtkinter.CTkFrame(app,height = 400, width = 550, corner_radius=8, fg_color=UI_SURFACE, border_width=1, border_color=UI_BORDER)
        app.program_frame.grid(row=1, column=1, columnspan=2, padx=(5,0), pady=5, sticky="nsew")
        app.program_frame.grid_columnconfigure(0, weight=1)
        app.program_frame.grid_rowconfigure(1, weight=0)
        app.program_frame.grid_rowconfigure(1, weight=1)

        app.program_label = customtkinter.CTkLabel(app.program_frame, text="Program:", font=customtkinter.CTkFont(size=16))
        app.program_label.grid(row=0, column=0, padx=(10,10), pady=5, sticky="w")

        app.textbox_program = customtkinter.CTkTextbox(app.program_frame, font=customtkinter.CTkFont(size=15, family='JetBrains Mono'), fg_color=UI_SURFACE_LOW, text_color=UI_ON_SURFACE, border_color=UI_BORDER, border_width=1, corner_radius=6)
        app.textbox_program.grid(row=1, column=0,columnspan=2, padx=(20, 20), pady=(5, 20), sticky="nsew")

        app.textbox_program.bind("<KeyRelease>", highlight_words_program)


    def start_stop_frame():
        app.start_stop_frame = customtkinter.CTkFrame(app,height = 30, corner_radius=8, fg_color=UI_SURFACE, border_width=1, border_color=UI_BORDER)
        app.start_stop_frame.grid(row=2, column=1, columnspan=2, padx=(5,0), pady=5, sticky="nsew")
        app.start_stop_frame.grid_columnconfigure(0, weight=0)
        #app.response_frame.grid_rowconfigure(1, weight=0)
        #app.response_frame.grid_rowconfigure(1, weight=1)

        _btn_font = customtkinter.CTkFont(family='Inter', size=15, weight='bold')
        app.start = customtkinter.CTkButton(app.start_stop_frame,text="Run",width= 50, font=_btn_font, fg_color=UI_SUCCESS, hover_color="#1ea800", text_color="#ffffff", command = execute_program)
        app.start.grid(row=0, column=0, padx=2,pady = 5,sticky="w")

        app.pause_resume = customtkinter.CTkButton(app.start_stop_frame,text="Pause", width= 55, font=_btn_font, fg_color=UI_WARN, hover_color="#cc9300", text_color="#241a00", command = pause_resume_program)
        app.pause_resume.grid(row=0, column=1, padx=2,pady = 5,sticky="w")

        app.stop = customtkinter.CTkButton(app.start_stop_frame,text="Stop", width= 50, font=_btn_font, fg_color=UI_DANGER, hover_color="#b71c1c", text_color="#ffffff", command = stop_program)
        app.stop.grid(row=0, column=2, padx=2,pady = 5,sticky="w")

        app.step = customtkinter.CTkButton(app.start_stop_frame,text="Step", width= 50, font=_btn_font, command = step_program)
        app.step.grid(row=0, column=3, padx=2,pady = 5,sticky="w")

        app.save = customtkinter.CTkButton(app.start_stop_frame,text="Save",width= 50, font=_btn_font, command = save_txt)
        app.save.grid(row=0, column=4, padx=2,pady = 5,sticky="w")

        app.save_as = customtkinter.CTkButton(app.start_stop_frame,text="Save as", width= 35,font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command = save_as_txt)
        app.save_as.grid(row=0, column=5, padx=2,pady = 5,sticky="w")

        app.open = customtkinter.CTkButton(app.start_stop_frame,text="Open",width= 35, font = customtkinter.CTkFont(size=15, family='TkDefaultFont'),command = open_txt)
        app.open.grid(row=0, column=6, padx=2,pady = 5,sticky="w")


    # dodaj slikice kao iz meca studio
    def response_log_frames():
        #response log frame
        app.response_frame = customtkinter.CTkFrame(app,height = 100, corner_radius=8, fg_color=UI_SURFACE, border_width=1, border_color=UI_BORDER)
        app.response_frame.grid(row=3, column=1, columnspan=2, padx=(5,0), pady=5, sticky="nsew")
        app.response_frame.grid_columnconfigure(0, weight=1)
        app.response_frame.grid_rowconfigure(1, weight=0)
        app.response_frame.grid_rowconfigure(1, weight=1)


        app.program_label = customtkinter.CTkLabel(app.response_frame, text="Response log:", font=customtkinter.CTkFont(size=16))
        app.program_label.grid(row=0, column=0, padx=(10,10), pady=5, sticky="w")

        app.Show_rec_frame = customtkinter.CTkButton(app.response_frame,text="Show received frame", font = customtkinter.CTkFont(size=16, family='TkDefaultFont'))
        app.Show_rec_frame.grid(row=0, column=1, padx=20,pady = 5,sticky="w")

        app.textbox_response = customtkinter.CTkTextbox(app.response_frame, font=customtkinter.CTkFont(size=14, family='JetBrains Mono'), fg_color=UI_SURFACE_LOW, text_color=UI_ON_SURFACE, border_color=UI_BORDER, border_width=1, corner_radius=6)
        app.textbox_response.grid(row=1, column=0,columnspan=2, padx=(20, 20), pady=(5, 20), sticky="nsew")
        app.textbox_response.bind("<KeyRelease>", highlight_words_response)


    #app.textbox_program.bind("<KeyRelease>", highlight_words)

    def commands_frames():
        #commands frame
        app.commands_frame = customtkinter.CTkFrame(app,height = 100,width = right_frames_width, corner_radius=8, fg_color=UI_SURFACE, border_width=1, border_color=UI_BORDER)
        app.commands_frame.grid(row=1, column=3, rowspan = 3, padx=(5,5), pady=5, sticky="news")
        app.commands_frame.grid_columnconfigure(0, weight=0)

        app.select_current_position = customtkinter.CTkRadioButton(master=app.commands_frame, text="Current position/Pose",  value=2,command = Current_position)
        app.select_current_position.grid(row=0, column=0, pady=10, padx=20, sticky="we")

        app.select_custom_position = customtkinter.CTkRadioButton(master=app.commands_frame, text="Custom positon/Pose",  value=2,command = Custom_position)
        app.select_custom_position.grid(row=1, column=0, pady=10, padx=20, sticky="we")

        app.select_current_position.select()
        #app.commands_frame.grid_propagate(False)
        # https://github.com/TomSchimansky/CustomTkinter/discussions/431
        style = ttk.Style()
        style.theme_use("default")

        style.configure("Treeview",
                        background=UI_SURFACE_LOW,
                        foreground=UI_ON_SURFACE,
                        rowheight=40,
                        fieldbackground=UI_SURFACE_LOW,
                        bordercolor=UI_BORDER,
                        borderwidth=0,
                        font=('JetBrains Mono',13))

        style.map('Treeview',
                  background=[('selected', UI_ACCENT)],
                  foreground=[('selected', '#ffffff')])

        style.configure("Treeview.Heading",
                        background=UI_ACCENT,
                        foreground="#ffffff",
                        relief="flat",
                        font=('Inter',13,'bold'),
                        )


        style.map("Treeview.Heading",
                    background=[('active', UI_ACCENT_DEEP)])

        app.add_menu_display211 = customtkinter.CTkFrame(master=app.commands_frame,
                                                    corner_radius=15,
                                                    height=100,
                                                    width=0)
        
        app.add_menu_display211.grid(pady=3, padx=5, sticky="nws")

        columns = ( 'item')

        app.table = ttk.Treeview(master=app.add_menu_display211,
                        columns=columns,
                        height=30,
                        selectmode='browse',
                        show='headings')
        
        app.table.column("#0", anchor="c", minwidth=200, width=250)
        app.table.column("#1", anchor="c", minwidth=200, width=250)

        app.table.heading('#0', text='test')
        app.table.heading('#1', text='Commands')
        

        #Commands_list = ["Home","Delay","End","Loop","IO","JointVelSet","JointAccSet","JointMove","PoseMove","JointVelMove",
                            #"CartAccSet","CartVelSet","CartLinVelSet","CartAngVelSet","CartMove","CartVelMoveTRF","CartVelMoveWRF"]
        # Commands
        Joint_space = app.table.insert(parent = '', index ='end',iid = 0,text="Parent",values="Joint_space")
        Cart_space = app.table.insert(parent = '', index ='end',iid = 1,text="Parent",values="Cartesian_space")
        Conditional_statements = app.table.insert(parent = '', index ='end',iid = 2,text="Parent",values="Conditional_stetements")
        Home = app.table.insert(parent = '', index ='end',iid = 3,text="Parent",values="Home")
        Delay = app.table.insert(parent = '', index ='end',iid = 4,text="Parent",values="Delay")
        End = app.table.insert(parent = '', index ='end',iid = 5,text="Parent",values="End")
        Loop = app.table.insert(parent = '', index ='end',iid = 6,text="Parent",values="Loop")
        Begin_ = app.table.insert(parent = '', index ='end',iid = 7,text="Parent",values="Begin")
        Input_var_ = app.table.insert(parent = '', index ='end',iid = 8,text="Parent",values="Input")
        Output_var_ = app.table.insert(parent = '', index ='end',iid = 9,text="Parent",values="Output")
        Gripper = app.table.insert(parent = '', index ='end',iid = 10,text="Parent",values="Gripper")
        Gripper_cal = app.table.insert(parent = '', index ='end',iid = 11,text="Parent",values="Gripper_cal")
        Get_data = app.table.insert(parent = '', index ='end',iid = 12,text="Parent",values="Get_data")
        Timeouts = app.table.insert(parent = '', index ='end',iid = 13,text="Parent",values="Timeouts")
        Vision_cmds = app.table.insert(parent = '', index ='end',iid = 14,text="Parent",values="Vision")
        Modbus_cmds = app.table.insert(parent = '', index ='end',iid = 15,text="Parent",values="Modbus")
        Research_cmds = app.table.insert(parent = '', index ='end',iid = 16,text="Parent",values="Research")

        # Joint space commands
        v1 = app.table.insert(Joint_space, index ='end',iid = 100,open = True, text="Child",values="MoveJoint")
        v2 = app.table.insert(Joint_space, index ='end',iid = 101,open = True, text="Child",values="MovePose")
        v3 = app.table.insert(Joint_space, index ='end',iid = 102,open = True, text="Child",values="SpeedJoint")

    
        # Cart space commands
        v1 = app.table.insert(Cart_space, index ='end',iid = 110,open = True, text="Child",values="MoveCart")
        v2 = app.table.insert(Cart_space, index ='end',iid = 120,open = True, text="Child",values="MoveCartRelTRF")
        v3 = app.table.insert(Cart_space, index ='end',iid = 130,open = True, text="Child",values="SpeedCart")

        app.table.insert(Vision_cmds, index ='end',iid = 140,open = True, text="Child",values="vision")
        app.table.insert(Modbus_cmds, index ='end',iid = 150,open = True, text="Child",values="ModbusRead")
        app.table.insert(Modbus_cmds, index ='end',iid = 151,open = True, text="Child",values="ModbusWrite")
        app.table.insert(Research_cmds, index ='end',iid = 152,open = True, text="Child",values="timestamp")

        app.table.grid(row=2, column=0, sticky='nsew', padx=8, pady=10)

        def select(e):
            selected = app.table.focus()
            logging.debug(selected)
            value = app.table.item(selected,'values')
            logging.debug(value[0])
            if value[0] == "Cartesian_space" or value[0] == "Joint_space" or value[0] == "Conditional_stetements" or value[0] == "Vision" or value[0] == "Modbus" or value[0] == "Research":
                None
                # Do nothing here because these are the selection menus
            elif Current_Custom_pose_select == "Current":
                if value[0] == "MoveJoint":
                    app.textbox_program.insert(tk.INSERT, str(value[0]) + "(" + Joint1_value + "," + Joint2_value + "," +Joint3_value + "," +
                                               Joint4_value +"," + Joint5_value +"," + Joint6_value + ")" +"\n")                    

                elif value[0] == "MovePose":
                    app.textbox_program.insert(tk.INSERT, str(value[0]) + "(" + x_value + "," + y_value + "," +z_value + "," +
                                               Rx_pos +"," + Ry_pos +"," + Rz_pos + ")" +"\n")   

                elif value[0] == "MoveCart":
                    app.textbox_program.insert(tk.INSERT, str(value[0]) + "(" + x_value + "," + y_value + "," +z_value + "," +
                                               Rx_pos +"," + Ry_pos +"," + Rz_pos + ")" +"\n")   
                    
                elif value[0] == "MoveCartRelTRF":
                    app.textbox_program.insert(tk.INSERT, str(value[0]) + "(" + str(0) + "," + str(0) + "," +str(0) + "," +
                                str(0) +"," + str(0) +"," + str(0) + ")" +"\n")   
                elif value[0] == "vision":
                    app.textbox_program.insert(tk.INSERT, "vision()" + "\n")
                elif value[0] == "ModbusRead":
                    app.textbox_program.insert(tk.INSERT, "ModbusRead(trigger_pick, HIGH, timeout=5)" + "\n")
                elif value[0] == "ModbusWrite":
                    app.textbox_program.insert(tk.INSERT, "ModbusWrite(cycle_done, HIGH)" + "\n")
                elif value[0] == "timestamp":
                    app.textbox_program.insert(tk.INSERT, "timestamp(label=\"event\", record)" + "\n")
                else:
                    app.textbox_program.insert(tk.INSERT, str(value[0]) + "()" +"\n")
          
            elif Current_Custom_pose_select == "Custom":
                if value[0] == "ModbusRead":
                    app.textbox_program.insert(tk.INSERT, "ModbusRead(trigger_pick, HIGH, timeout=5)" + "\n")
                elif value[0] == "ModbusWrite":
                    app.textbox_program.insert(tk.INSERT, "ModbusWrite(cycle_done, HIGH)" + "\n")
                elif value[0] == "timestamp":
                    app.textbox_program.insert(tk.INSERT, "timestamp(label=\"event\", record)" + "\n")
                else:
                    app.textbox_program.insert(tk.INSERT, str(value[0]) + "()" +"\n")

        app.table.bind('<ButtonRelease-1>', select)


    def Change_gripper_ID():
        ID_var = app.grip_ID_entry.get()
        if(int(ID_var) > 16 ):
            None
        else:
            Gripper_data_out[5] = int(ID_var)

    def Set_gripper_pos():
        pos_value = app.grip_pos_entry.get()
        if(pos_value != ""):
            if(int(pos_value) >= 0 and int(pos_value)<= 255):
                print(pos_value)
                app.grip_pos_slider.set(int(pos_value))
                #Gripper_data_out[4] = 0

    def Set_gripper_vel():
        
        vel_value = app.grip_speed_entry.get()
        if(vel_value != ""):
            if(int(vel_value) >= 0 and int(vel_value)<= 255):
                print(vel_value)
                app.grip_speed_slider.set(int(vel_value))

    def Set_gripper_cur():
        cur_value = app.grip_current_entry.get()
        if(cur_value != ""):
            if(int(cur_value) >= 0 and int(cur_value)<= 1000):
                print(cur_value)
                app.grip_current_slider.set(int(cur_value))

    def Gripper_set_values():
        Gripper_data_out[0] = int(app.grip_pos_slider.get())
        Gripper_data_out[1] = int(app.grip_speed_slider.get())
        Gripper_data_out[2] = int(app.grip_current_slider.get())


    def Gripper_calibrate():
        Gripper_data_out[4] = 1

    def Gripper_clear_error():
        Gripper_data_out[4] = 2

    def Set_output_1(state):
        logging.debug("Output 1 state is: ")
        logging.debug(state)
        InOut_out[2] = state
        InOut_in[2] = state

    def Set_output_2(state):
        logging.debug("Output 2 state is: ")
        logging.debug(state)
        InOut_out[3] = state
        InOut_in[3] = state

    def Enable_press():
        Buttons[1] = 1
        logging.debug("Enable press")

    def Disable_press():
        Buttons[2] = 1
        logging.debug("Disable press")

    def Set_comm_port():

        COMPORT_value = app.COMPORT.get()
        
        # For macOS, allow full port paths
        if my_os == "Darwin":
            # If input looks like a full path, store it as a string in General_data[2]
            if COMPORT_value.startswith('/dev/'):
                # Store the full path in a new General_data index for macOS
                if len(General_data) < 3:
                    # Extend General_data array if needed
                    General_data.extend([0])  # Add space for full port path flag
                General_data[0] = -1  # Use -1 to indicate full path mode
                # Store full path in shared_string temporarily for serial functions to access
                shared_string.value = COMPORT_value.encode('utf-8')
            else:
                # Try to extract number for backward compatibility
                pattern = re.compile(r'\D*(\d+)\D*')
                match = pattern.match(COMPORT_value)
                if match:
                    com_number = int(match.group(1))
                    General_data[0] = com_number
                else:
                    # Default to 0 if no match found
                    General_data[0] = 0
        else:
            # For Windows and Linux, keep existing behavior
            pattern = re.compile(r'\D*(\d+)\D*')
            match = pattern.match(COMPORT_value)
            if match:
               com_number = int(match.group(1))
               General_data[0] = com_number
            else:
                 General_data[0] = 0  # Default to 0 if no match found

        print("Port setting:", General_data[0])
        if my_os == "Darwin" and General_data[0] == -1:
            print("Full path mode for macOS:", COMPORT_value)
        
        

    def Clear_error():
        Buttons[3] = 1
        logging.debug("Clear error")

    def _left_workspace_frames():
        return [
            app.left_scrollable_container,
        ]

    def _center_workspace_frames():
        return [
            app.program_frame,
            app.start_stop_frame,
            app.response_frame,
        ]

    def _right_workspace_frames():
        return [app.commands_frame]

    def _bottom_workspace_frames():
        return [app.bottom_select_frame]

    def _standard_workspace_frames():
        return _left_workspace_frames() + _center_workspace_frames() + _right_workspace_frames() + _bottom_workspace_frames()

    def _feature_workspace_frames():
        return [app.vision_frame, app.modbus_frame, app.research_frame]

    def _layout_handles():
        return [
            getattr(app, "left_resize_handle", None),
            getattr(app, "right_resize_handle", None),
            getattr(app, "bottom_resize_handle", None),
        ]

    def _active_is_feature():
        return getattr(app, "current_menu", "") in ["Vision", "Modbus", "Research"]

    def _refresh_layout_buttons():
        pass

    def _configure_panel_sizes():
        left_width = 0 if app.layout_collapsed["left"] else app.layout_sizes["left"]
        right_width = 0 if app.layout_collapsed["right"] else app.layout_sizes["right"]
        bottom_height = 0 if app.layout_collapsed["bottom"] else app.layout_sizes["bottom"]

        app.grid_columnconfigure(0, minsize=left_width)
        app.grid_columnconfigure(3, minsize=right_width)
        app.grid_rowconfigure(4, minsize=bottom_height)

        for frame in _left_workspace_frames():
            frame.configure(width=max(left_width, 1))
            if not isinstance(frame, customtkinter.CTkScrollableFrame):
                frame.grid_propagate(False)
        for frame in _right_workspace_frames():
            frame.configure(width=max(right_width, 1))
            if not isinstance(frame, customtkinter.CTkScrollableFrame):
                frame.grid_propagate(False)
        for frame in _bottom_workspace_frames():
            frame.configure(height=max(bottom_height, 1))
            if not isinstance(frame, customtkinter.CTkScrollableFrame):
                frame.grid_propagate(False)

    def _place_resize_handles():
        if _active_is_feature():
            for handle in _layout_handles():
                if handle is not None:
                    handle.grid_remove()
            return

        if not app.layout_collapsed["left"] and hasattr(app, "left_resize_handle"):
            app.left_resize_handle.grid(row=1, column=0, rowspan=3, padx=(0,0), pady=8, sticky="nse")
            app.left_resize_handle.tkraise()
        elif hasattr(app, "left_resize_handle"):
            app.left_resize_handle.grid_remove()

        if not app.layout_collapsed["right"] and hasattr(app, "right_resize_handle"):
            app.right_resize_handle.grid(row=1, column=3, rowspan=3, padx=(0,0), pady=8, sticky="nsw")
            app.right_resize_handle.tkraise()
        elif hasattr(app, "right_resize_handle"):
            app.right_resize_handle.grid_remove()

        if not app.layout_collapsed["bottom"] and hasattr(app, "bottom_resize_handle"):
            app.bottom_resize_handle.grid(row=4, column=0, columnspan=4, padx=12, pady=(0,0), sticky="new")
            app.bottom_resize_handle.tkraise()
        elif hasattr(app, "bottom_resize_handle"):
            app.bottom_resize_handle.grid_remove()

    def _show_standard_workspace():
        _configure_panel_sizes()
        for feature_frame in _feature_workspace_frames():
            feature_frame.grid_remove()

        if app.layout_collapsed["left"]:
            for left_frame in _left_workspace_frames():
                left_frame.grid_remove()
        else:
            for left_frame in _left_workspace_frames():
                left_frame.grid()

        for center_frame in _center_workspace_frames():
            center_frame.grid()

        if app.layout_collapsed["right"]:
            for right_frame in _right_workspace_frames():
                right_frame.grid_remove()
        else:
            for right_frame in _right_workspace_frames():
                right_frame.grid()

        if app.layout_collapsed["bottom"]:
            for bottom_frame in _bottom_workspace_frames():
                bottom_frame.grid_remove()
        else:
            for bottom_frame in _bottom_workspace_frames():
                bottom_frame.grid()

        _place_resize_handles()
        _refresh_layout_buttons()

    def _raise_current_commander_frame():
        active_frame = {
            "Jog": app.jog_frame,
            "Cart": app.cart_frame,
            "I/O": app.IO_frame,
            "Settings": getattr(app, "settings_frame", None),
            "Calibrate": app.Calibrate_frame,
            "Gripper": app.Gripper_frame,
        }.get(getattr(app, "current_menu", ""), app.jog_frame)
        if active_frame:
            for f in [app.jog_frame, app.cart_frame, app.IO_frame, app.Calibrate_frame, app.Gripper_frame]:
                f.grid_remove()
            active_frame.grid()
            active_frame.tkraise()
        _place_resize_handles()

    def toggle_layout_panel(panel):
        app.layout_collapsed[panel] = not app.layout_collapsed[panel]
        if _active_is_feature():
            _refresh_layout_buttons()
            return
        _show_standard_workspace()
        _raise_current_commander_frame()

    def layout_resize_handles():
        app.left_resize_handle = customtkinter.CTkFrame(app, width=7, corner_radius=3, fg_color=UI_HANDLE, cursor="sb_h_double_arrow")
        app.right_resize_handle = customtkinter.CTkFrame(app, width=7, corner_radius=3, fg_color=UI_HANDLE, cursor="sb_h_double_arrow")
        app.bottom_resize_handle = customtkinter.CTkFrame(app, height=7, corner_radius=3, fg_color=UI_HANDLE, cursor="sb_v_double_arrow")

        def start_resize(panel, event):
            app.resize_state = {
                "panel": panel,
                "x": event.x_root,
                "y": event.y_root,
                "left": app.layout_sizes["left"],
                "right": app.layout_sizes["right"],
                "bottom": app.layout_sizes["bottom"],
            }

        def drag_resize(event):
            panel = app.resize_state.get("panel")
            if panel == "left":
                value = app.resize_state["left"] + (event.x_root - app.resize_state["x"])
                app.layout_sizes["left"] = max(280, min(760, int(value)))
            elif panel == "right":
                value = app.resize_state["right"] - (event.x_root - app.resize_state["x"])
                app.layout_sizes["right"] = max(220, min(560, int(value)))
            elif panel == "bottom":
                value = app.resize_state["bottom"] - (event.y_root - app.resize_state["y"])
                app.layout_sizes["bottom"] = max(64, min(180, int(value)))
            _show_standard_workspace()

        for handle, panel in [
            (app.left_resize_handle, "left"),
            (app.right_resize_handle, "right"),
            (app.bottom_resize_handle, "bottom"),
        ]:
            handle.bind("<ButtonPress-1>", lambda event, p=panel: start_resize(p, event))
            handle.bind("<B1-Motion>", drag_resize)

    def _show_commander_frame(frame, name):
        app.current_menu = name
        _show_standard_workspace()
        for f in [app.jog_frame, app.cart_frame, app.IO_frame, app.Calibrate_frame, app.Gripper_frame]:
            f.grid_remove()
        frame.grid()
        frame.tkraise()
        _place_resize_handles()
        _update_tab_highlights(name)
        logging.debug(name)

    def _show_feature_frame(frame, name):
        app.current_menu = name
        for standard_frame in _standard_workspace_frames():
            standard_frame.grid_remove()
        for handle in _layout_handles():
            if handle is not None:
                handle.grid_remove()
        for feature_frame in _feature_workspace_frames():
            feature_frame.grid_remove()
        frame.grid(row=1, column=0, columnspan=4, rowspan=4, padx=(5,5), pady=5, sticky="news")
        frame.tkraise()
        _refresh_layout_buttons()
        _update_tab_highlights(name)
        logging.debug(name)

    def raise_frame_cart():
        _show_commander_frame(app.jog_frame, "Cart")
        app.cart_frame.tkraise()
        _place_resize_handles()
        
    def raise_frame_jog():
        _show_commander_frame(app.jog_frame, "Jog")

    def WRF_button():
        app.TRF_select.deselect()
        global Wrf_Trf
        Wrf_Trf = "WRF"
        Jog_control[2] = 1
        logging.debug(Wrf_Trf)

    def TRF_button():
        app.WRF_select.deselect()
        global Wrf_Trf
        Wrf_Trf = "TRF"
        Jog_control[2] = 0
        logging.debug(Wrf_Trf)

    def Current_position():
        app.select_custom_position.deselect()
        global Current_Custom_pose_select
        Current_Custom_pose_select = "Current"
        logging.debug(Current_Custom_pose_select)


    def Custom_position():
        app.select_current_position.deselect()
        global Current_Custom_pose_select
        Current_Custom_pose_select = "Custom"
        logging.debug(Current_Custom_pose_select)

    # save u neki temp file?
    def open_txt():

        global Now_open_txt
        logging.debug("Open txt")
        logging.debug(Now_open_txt)
        app.textbox_program.delete('1.0', tk.END)
        text_file = filedialog.askopenfilename(initialdir = Image_path + "/Programs",title = "open text file", filetypes= (("Text Files",".txt"),))
        logging.debug(text_file)
        Now_open_txt = text_file
        text_file = open(text_file,'r+')
        temp_var = text_file.read()
        app.textbox_program.insert(tk.END,temp_var)
        text_file.close()

    def execute_program():
        # When program start button is pressed:
        # save the current file and save its content to execute_script.txt file
        # set Button[7] flag to 1
        logging.debug("Execute program")
        global Now_open_txt
        logging.debug(Now_open_txt)
        # If program was blank or execute script
        if Now_open_txt == '' or Now_open_txt == Image_path + "/Programs/execute_script.txt":
            Now_open_txt = Image_path + "/Programs/execute_script.txt"
            text_file = open(Now_open_txt,'w+')
            text_file.write(app.textbox_program.get(1.0,tk.END))
            text_file.close()
        # If program was saved under some name.
        # Save it again with that name and transfer content of the file to the execute script.
        else:
            text_file = open(Now_open_txt,'w+')
            text_file.write(app.textbox_program.get(1.0,tk.END))
            text_file.close()
            x = Image_path + "/Programs/execute_script.txt"
            text_file = open(x,'w+')
            text_file.write(app.textbox_program.get(1.0,tk.END))
            text_file.close()
            
        # Set flag to 1. Program will try to run
        update_state("program_control", {"state": "RUNNING", "paused": False, "stop_requested": False, "step_requested": 0, "updated_at": time.time()})
        research_logger.record("program_run", 1, Now_open_txt)
        Buttons[7] = 1
        

    def stop_program():
        logging.debug("Stop program")
        update_state("program_control", {"state": "STOP_REQUESTED", "paused": False, "stop_requested": True, "step_requested": 0, "updated_at": time.time()})
        research_logger.record("program_stop_requested", 1)
        Buttons[7] = 0

    def pause_resume_program():
        control = read_state("program_control", {})
        paused = not bool(control.get("paused", False))
        state = "PAUSED" if paused else "RUNNING"
        update_state("program_control", {"state": state, "paused": paused, "stop_requested": False, "step_requested": 0, "updated_at": time.time()})
        app.pause_resume.configure(text="Resume" if paused else "Pause")
        research_logger.record("program_pause" if paused else "program_resume", 1)

    def step_program():
        execute_program()
        update_state("program_control", {"state": "STEP", "paused": False, "stop_requested": False, "step_requested": time.time(), "updated_at": time.time()})
        research_logger.record("program_step", 1)

    def save_txt():
        logging.debug("Save txt")
        global Now_open_txt
        logging.debug(Now_open_txt)
        if Now_open_txt != '':
            #print("done")
            text_file = open(Now_open_txt,'w+')
            text_file.write(app.textbox_program.get(1.0,tk.END))
            text_file.close()
        else:
            Now_open_txt = Image_path + "/Programs/execute_script.txt"
            text_file = open(Now_open_txt,'w+')
            text_file.write(app.textbox_program.get(1.0,tk.END))
            text_file.close() 
        
    def save_as_txt():
        logging.debug("Save as txt")

        file_path = filedialog.asksaveasfilename(defaultextension=".txt")
        if file_path:
            with open(file_path, "w") as file:
                content = app.textbox_program.get("1.0", tk.END)
                file.write(content)

    def Select_simulator():
        global Robot_sim
        Robot_sim = not Robot_sim 
        if(Robot_sim == 0):
            app.radio_button_sim.deselect()
            Buttons[5] = 0
        else:
            app.radio_button_sim.select()
            Buttons[5] = 1
        logging.debug(Robot_sim) 

    def Select_real_robot():
        global Real_robot
        Real_robot  = not Real_robot
        if(Real_robot == 0):
            app.radio_button_real.deselect()
            Buttons[4] = 0
        else:
            app.radio_button_real.select()
            Buttons[4] = 1
        logging.debug(Real_robot)


    def Select_gripper_activate():
        global Gripper_activate_deactivate
        Gripper_activate_deactivate  = not Gripper_activate_deactivate
        if(Gripper_activate_deactivate == 0):
            app.grip_activate_radio.deselect()
            #Buttons[4] = 0
        else:
            app.grip_activate_radio.select()
            #Buttons[4] = 1
        logging.debug(Gripper_activate_deactivate)

    def quick_gripper_button():
        global Quick_grip
        Quick_grip = not Quick_grip
        if(Quick_grip == 0):
            InOut_out[2] = 0
            app.Quick_gripper_on_off.deselect()
        else:
            InOut_out[2] = 1
            app.Quick_gripper_on_off.select()
        logging.debug("Quick grip status is: ")
        logging.debug(Quick_grip)

    def raise_frame_IO():
        _show_commander_frame(app.IO_frame, "I/O")

    def raise_calibrate_frame():
        _show_commander_frame(app.Calibrate_frame, "Calibrate")


    def raise_gripper_frame():
        _show_commander_frame(app.Gripper_frame, "Gripper")

    def raise_vision_frame():
        _show_feature_frame(app.vision_frame, "Vision")

    def raise_modbus_frame():
        _show_feature_frame(app.modbus_frame, "Modbus")

    def raise_research_frame():
        _show_feature_frame(app.research_frame, "Research")

    def show_warrning():
        messagebox.showwarning("test","test2")

    def show_error():
        messagebox.showerror("test","test2")

    def show_info():
        messagebox.showinfo("test","test2")

    def Home_robot():
        logging.debug("Home button pressed")
        Buttons[0] = 1

    def Park_robot():
        logging.debug("Park button pressed")
        Buttons[8] = 1
        app.textbox_program.insert(tk.INSERT, "Begin()"  +"\n") 
        app.textbox_program.insert(tk.INSERT, "MoveJoint(90.0,-144.683,108.171,2.222,25.003,180.0,t=4)" +"\n")     
        app.textbox_program.insert(tk.INSERT, "End()" +"\n") 

    def Open_help():
        messagebox.showwarning("test","test2")
        messagebox.showerror("test","test2")
        messagebox.showinfo("test","test2")
        logging.debug(Position_in)
    def change_appearance_mode_event(new_appearance_mode: str):
        customtkinter.set_appearance_mode(new_appearance_mode)

        if(new_appearance_mode == "Dark"):
            style = ttk.Style()
            style.theme_use("default")

            style.configure("Treeview",
                            background="#202630",
                            foreground="white",
                            rowheight=40,
                            fieldbackground="#202630",
                            bordercolor="#2f3844",
                            borderwidth=0,
                            font=('JetBrains Mono',13))

            style.map('Treeview',
                      background=[('selected', UI_ACCENT)],
                      foreground=[('selected', '#ffffff')])

            style.configure("Treeview.Heading",
                            background="#2f3844",
                            foreground="white",
                            relief="flat",
                            font=('Inter',13,'bold'),
                            )

            style.map("Treeview.Heading",
                        background=[('active', UI_ACCENT_DEEP)])

        if(new_appearance_mode == "Light"):
            style = ttk.Style()
            style.theme_use("default")

            style.configure("Treeview",
                            background=UI_SURFACE_LOW,
                            foreground=UI_ON_SURFACE,
                            rowheight=40,
                            fieldbackground=UI_SURFACE_LOW,
                            bordercolor=UI_BORDER,
                            borderwidth=0,
                            font=('JetBrains Mono',13))

            style.map('Treeview',
                      background=[('selected', UI_ACCENT)],
                      foreground=[('selected', '#ffffff')])

            style.configure("Treeview.Heading",
                            background=UI_ACCENT,
                            foreground="#ffffff",
                            relief="flat",
                            font=('Inter',13,'bold'),
                            )

            style.map("Treeview.Heading",
                        background=[('active', UI_ACCENT_DEEP)])



    # This function periodically updates elements of the GUI that need to be updated
    def Stuff_To_Update():
        
        #logging.debug("test")
        global prev_string_shared

        global x_value 
        global y_value 
        global z_value 
        global Rx_pos 
        global Ry_pos 
        global Rz_pos 

        global Joint1_value 
        global Joint2_value 
        global Joint3_value 
        global Joint4_value 
        global Joint5_value 
        global Joint6_value 

        shared_string_string = (shared_string.value).decode('utf-8')
        if shared_string_string != prev_string_shared:
            now = datetime.now()
            time_string = now.strftime("%H:%M:%S")
            prev_string_shared = shared_string_string

            app.textbox_response.insert(tk.INSERT,time_string + "--" + shared_string_string + "\n")
            app.textbox_response.see(tk.END)
        else: 
            prev_string_shared = shared_string_string

        #app.textbox_response.insert(tk.INSERT,"tesT\n")

        app.Input1.configure(app.IO_frame, text="INPUT 1: " + str(InOut_in[0]).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.Input2.configure(app.IO_frame, text="INPUT 2: " + str(InOut_in[1]).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.ESTOP_STATUS.configure(app.IO_frame, text="ESTOP: " + str(InOut_in[4]).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.OUTPUT_1_LABEL.configure(app.IO_frame, text="OUTPUT 1 is: " + str(InOut_out[2]).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        app.OUTPUT_2_LABEL.configure(app.IO_frame, text="OUTPUT 2 is: " + str(InOut_out[3]).rjust(7, ' '), font=customtkinter.CTkFont(size=text_size))
        if( InOut_in[4] == 0):
            app.estop_status.configure(app.bottom_select_frame, text="● ESTOP ACTIVE", text_color=UI_DANGER, font=customtkinter.CTkFont(family='Inter', size=15, weight='bold'))
        else:
            app.estop_status.configure(app.bottom_select_frame, text="● READY", text_color=UI_SUCCESS, font=customtkinter.CTkFont(family='Inter', size=15, weight='bold'))


    
    # Tool positions
    # Use ik to calculate 

        # Array of current joint positions in radians
        q1 = np.array([PAROL6_ROBOT.STEPS2RADS(Position_in[0],0),
                       PAROL6_ROBOT.STEPS2RADS(Position_in[1],1),
                       PAROL6_ROBOT.STEPS2RADS(Position_in[2],2),
                       PAROL6_ROBOT.STEPS2RADS(Position_in[3],3),
                       PAROL6_ROBOT.STEPS2RADS(Position_in[4],4),
                       PAROL6_ROBOT.STEPS2RADS(Position_in[5],5),])
        # Get SE3 matrix of current joint positions
        T = PAROL6_ROBOT.robot.fkine(q1)
        b = T.t # get translation component
        #print(b*1000)
        robot_pose[0] = b[0] * 1000 #  X in mm
        robot_pose[1] = b[1] * 1000 #  Y in mm
        robot_pose[2] = b[2] * 1000 #  Z in mm

        # - ``'xyz'``, rotate by yaw about the x-axis, then by pitch about the new y-axis,
        # then by roll about the new z-axis. Convention for a robot gripper with z-axis forward
        # and y-axis between the gripper fingers.
        robot_pose[3:] = T.rpy('deg','xyz') # get rotation component
  
        x_value = str(round(robot_pose[0],3))
        y_value = str(round(robot_pose[1],3))
        z_value = str(round(robot_pose[2],3))
        Rx_pos = str(round(robot_pose[3],3))
        Ry_pos = str(round(robot_pose[4],3))
        Rz_pos = str(round(robot_pose[5],3)) 

        app.x_pos.configure(text="X: "+ x_value.rjust(7, ' '))	
        app.y_pos.configure( text="Y: "+ y_value.rjust(7, ' '))	
        app.z_pos.configure(text="Z: "+ z_value.rjust(7, ' '))	
        app.Rx_pos.configure(text="Rx: "+ Rx_pos.rjust(7, ' '))	
        app.Ry_pos.configure(text="Ry: "+ Ry_pos.rjust(7, ' '))	
        app.Rz_pos.configure(text="Rz: "+ Rz_pos.rjust(7, ' '))	

    # Joint positions

        Joint1_value = str(round(PAROL6_ROBOT.STEPS2DEG(Position_in[0],0),3))
        Joint2_value = str(round(PAROL6_ROBOT.STEPS2DEG(Position_in[1],1),3))
        Joint3_value = str(round(PAROL6_ROBOT.STEPS2DEG(Position_in[2],2),3))
        Joint4_value = str(round(PAROL6_ROBOT.STEPS2DEG(Position_in[3],3),3))
        Joint5_value = str(round(PAROL6_ROBOT.STEPS2DEG(Position_in[4],4),3))
        Joint6_value = str(round(PAROL6_ROBOT.STEPS2DEG(Position_in[5],5),3))


        app.theta1.configure(text="θ1: " + Joint1_value.rjust(7, ' '))
        app.theta2.configure(text="θ2: " + Joint2_value.rjust(7, ' '))
        app.theta3.configure(text="θ3: " + Joint3_value.rjust(7, ' '))
        app.theta4.configure(text="θ4: " + Joint4_value.rjust(7, ' '))
        app.theta5.configure(text="θ5: " + Joint5_value.rjust(7, ' '))
        app.theta6.configure(text="θ6: " + Joint6_value.rjust(7, ' '))
        
        prev_positions[0] = Position_in[0]
        prev_positions[1] = Position_in[1]
        prev_positions[2] = Position_in[2]
        prev_positions[3] = Position_in[3]
        prev_positions[4] = Position_in[4]
        prev_positions[5] = Position_in[5]

        # Sliders
        # Velocity slider
        v1 = app.slider1.get()
        Jog_control[0] = int(v1)
        # Acc slider
        v2 = app.slider2.get()
        Jog_control[1] = int(v2)
        #Jog_control[0] = v1
        #Jog_control[1] = v2
        #logging.debug(Jog_control)
        app.Velocity_percent.configure(text= ""+ str(v1).rjust(4, ' ')+ "%")
        app.Accel_percent.configure(text= "" + str(v2).rjust(4, ' ') + "%")

        # Gripper stuff
        # Sliders
        gpos = app.grip_pos_slider.get()
        #Gripper_data_out[0] = int(gpos)
        app.grip_pos_percent.configure(text= ""+ str(gpos))

        gvel = app.grip_speed_slider.get()
        #Gripper_data_out[1] = int(gvel)
        app.grip_speed_percent.configure(text= ""+ str(gvel))
        
        gcur = app.grip_current_slider.get()
        #Gripper_data_out[2] = int(gcur)
        app.grip_current_percent.configure(text= "" + str(gcur).rjust(0, ' ') + " mA")

        app.grip_feedback_pos.configure(text="Gripper position feedback is: " + str(round(Gripper_data_in[1],0)).rjust(7, ' '))
        app.grip_feedback_current.configure(text="Gripper current feedback is: " + str(round(Gripper_data_in[3],0)).rjust(7, ' '))
        app.grip_object_detection.configure(text="Gripper object detection is: " + str(round(Gripper_data_in[4],0)).rjust(7, ' '))
       
        #bitfield_list = [Gripper_activate_deactivate,Gripper_action_status,InOut_in[4],Gripper_rel_dir,0,0,0,0] #InOut_in[4] is estop
        bitfield_list = [Gripper_activate_deactivate,Gripper_action_status,not InOut_in[4],Gripper_rel_dir,0,0,0,0] #InOut_in[4] is estop
        fused = PAROL6_ROBOT.fuse_bitfield_2_bytearray(bitfield_list)
        Gripper_data_out[3] = int(fused.hex(),16)

        Gripper_data_byte = PAROL6_ROBOT.split_2_bitfield(Gripper_data_in[4]) 
        fused_number = (Gripper_data_byte[2] << 1) | Gripper_data_byte[3]
        if(fused_number == 0):
            app.grip_object_detection.configure(text="Gripper in motion ")
        elif(fused_number == 1):
            app.grip_object_detection.configure(text="Object detected when closing ")
        elif(fused_number == 2):
            app.grip_object_detection.configure(text="Object detected when opening ")
        elif(fused_number == 3):
            app.grip_object_detection.configure(text="Gripper is at position ")

        app.grip_cal_status.configure(text="Calibration status is: " + str(Gripper_data_byte[7]).rjust(7, ' '))
        app.Error_status_grip.configure(text="Error status is: " + str(Gripper_data_byte[6]).rjust(7, ' '))
        app.Gripper_ID.configure(text="Gripper ID is: " + str(Gripper_data_out[5]))

        highlight_words_response(None)
        highlight_words_program(None)
        _update_vision_ui(read_state("vision", {}))
        _update_modbus_ui(read_state("modbus", {}))
        _update_research_ui(read_state("research", {}))
        # If tab is joint jog
        # Update joint sliders
        for y in range(0,6):
            app.progress_bar_joints[y].set(np.interp(Position_in[y],[PAROL6_ROBOT.Joint_limits_steps[y][0],PAROL6_ROBOT.Joint_limits_steps[y][1]],[0.0,1.0]))
            None
    
        app.after(66,Stuff_To_Update) # Update data every 66 ms ( 15 frames per second)



    #Stuff_To_Update(self)
    top_frames()
    bottom_frames()
    robot_positions_frames()
    program_frames()
    response_log_frames()
    commands_frames()
    cart_jog_frame()
    joint_jog_frames()
    IO_frame()
    start_stop_frame()
    Calibrate_frame()
    Gripper_frame()
    vision_frame()
    modbus_frame()
    research_frame()
    layout_resize_handles()
    _show_commander_frame(app.jog_frame, "Jog")

    Stuff_To_Update()



    app.mainloop() 


    


if __name__ == "__main__":

    #DUMMY DATA
    
    # Data sent by the PC to the robot
    Position_out = [1,11,111,1111,11111,10]
    Speed_out = [2,21,22,23,24,25]
    Command_out = 69
    Affected_joint_out = [1,1,1,1,1,1,1,1]
    InOut_out = [1,1,1,1,1,1,1,1]
    Timeout_out = 123

    # Data we send to the gripper!
    # Positon,speed,current,command,mode,ID
    # We use Positon,speed,current,command, mode(used for sending calibration)
    # Command is 8 bits fused
    Gripper_data_out = [1,1,1,1,1,0]

    # Data sent from robot to PC
    Position_in = [31,32,33,34,35,36]
    Speed_in = [41,42,43,44,45,46]
    Homed_in = [1,1,1,1,1,1,1,1]
    InOut_in = [1,1,1,1,1,1,1,1]
    Temperature_error_in = [1,1,1,1,1,1,1,1]
    Position_error_in = [1,1,1,1,1,1,1,1]
    Timeout_error = 123
    # how much time passed between 2 sent commands (2byte value, last 2 digits are decimal so max value is 655.35ms?)
    Timing_data_in = 123
    XTR_data =   123

    # Data we get from the gripper
    #ID,Position,speed,current,status,obj_detection
    # From this data we will use: position, current, status 
    Gripper_data_in = [110,120,130,140,150,160]

    # GUI control data
    Joint_jog_buttons = [0,0,0,0,0,0,0,0,0,0,0,0]
    Cart_jog_buttons = [0,0,0,0,0,0,0,0,0,0,0,0]
    # Speed slider, acc slider, WRF/TRF
    Jog_control = [0,0,0,0]
    # COM PORT, BAUD RATE, 
    General_data = [8,3000000]
    # Home,Enable,Disable,Clear error,Real_robot,Sim_robot,Demo app,Program executions,Park
    Buttons = [0,0,0,0,1,1,0,0,0]
    
    shared_string = multiprocessing.Array('c', b' ' * 100)

    GUI(shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons)
