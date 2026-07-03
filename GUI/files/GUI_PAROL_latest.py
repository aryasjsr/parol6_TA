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
from PIL import Image, ImageDraw, ImageTk
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

text_size = 13
PROGRAM_TEXT_FONT_SIZE = 16
LOG_TEXT_FONT_SIZE = 16
COMMAND_TREE_FONT_SIZE = 11
COMMAND_HELP_FONT_SIZE = 13

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


def _set_text(widget, text):
    """Update a widget's text only when it actually changed.

    Every customtkinter widget is a Canvas that repaints its rounded corners on
    each ``configure``. Stuff_To_Update runs ~15x/s, so re-applying identical
    text dozens of times per tick caused needless redraws (very noticeable when
    the window is maximized/fullscreen). Caching the last value skips that work.
    """
    if getattr(widget, "_last_text_cache", None) != text:
        widget._last_text_cache = text
        widget.configure(text=text)


#customtkinter.set_appearance_mode("Light")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_appearance_mode("Dark")  # Industrial Precision HMI — dark variant
customtkinter.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"

# --- Unified Modern Typography System — Montserrat ---
FONT_FAMILY_MAIN = "Montserrat"
FONT_FAMILY_MONO = "Consolas" if my_os == "Windows" else "monospace"
_fonts_resolved = False

def _lazy_resolve_fonts():
    """Resolve best available system fonts lazily (only when a Tk root already exists)."""
    global FONT_FAMILY_MAIN, FONT_FAMILY_MONO, _fonts_resolved
    if _fonts_resolved:
        return
    _fonts_resolved = True
    try:
        import tkinter.font as tkfont
        available = [f.lower() for f in tkfont.families()]
        # Montserrat is the primary font — fall back gracefully if not installed
        for sans in ["montserrat", "inter", "segoe ui", "ubuntu", "liberation sans", "dejavu sans", "helvetica", "arial"]:
            if sans in available:
                idx = available.index(sans)
                FONT_FAMILY_MAIN = list(tkfont.families())[idx]
                break
        # Best available monospace fonts
        for mono in ["jetbrains mono", "consolas", "ubuntu mono", "liberation mono", "dejavu sans mono", "courier new"]:
            if mono in available:
                idx = available.index(mono)
                FONT_FAMILY_MONO = list(tkfont.families())[idx]
                break
    except Exception:
        pass

_orig_CTkFont = customtkinter.CTkFont

# --- Clean Font Size Scale ---
# Defines a clear typographic hierarchy for professional UI readability.
#   Display/Title : 18px  (section headings, hero labels)
#   Heading       : 15px  (panel titles, group headers)
#   Subheading    : 13px  (button labels, prominent UI text)
#   Body          : 12px  (default body text)
#   Caption       : 11px  (helper text, status labels)
#   Small         : 10px  (footnotes, micro-labels)

class UnifiedCTkFont(_orig_CTkFont):
    def __init__(self, family=None, size=None, weight=None, slant=None, underline=None, overstrike=None):
        # Lazily resolve system fonts on first use (Tk root already exists at this point)
        _lazy_resolve_fonts()

        # 1. Map requested font families → Montserrat (main) or monospace
        if family is None or family == 'TkDefaultFont' or family == 'Segoe UI Symbol':
            family_mapped = FONT_FAMILY_MAIN
        elif family in ('JetBrains Mono', 'monospace', 'Consolas', 'Liberation Mono', 'DejaVu Sans Mono', 'Courier New'):
            family_mapped = FONT_FAMILY_MONO
        elif family in ('Inter', 'Segoe UI', 'Helvetica', 'Arial', 'Ubuntu'):
            # Redirect legacy font requests to Montserrat
            family_mapped = FONT_FAMILY_MAIN
        else:
            family_mapped = family

        # 2. Rescale font sizes to a clean, professional hierarchy
        if size is not None:
            if size >= 24:
                size_mapped = 18   # Display
            elif size >= 18:
                size_mapped = 15   # Heading
            elif size >= 15:
                size_mapped = 13   # Subheading
            elif size >= 14:
                size_mapped = 12   # Body
            elif size >= 12:
                size_mapped = 11   # Caption
            elif size >= 10:
                size_mapped = 10   # Small
            else:
                size_mapped = int(size)
        else:
            size_mapped = 12  # Default body size

        kwargs = {}
        if family_mapped is not None:
            kwargs['family'] = family_mapped
        if size_mapped is not None:
            kwargs['size'] = size_mapped
        if weight is not None:
            kwargs['weight'] = weight
        if slant is not None:
            kwargs['slant'] = slant
        if underline is not None:
            kwargs['underline'] = underline
        if overstrike is not None:
            kwargs['overstrike'] = overstrike
        super().__init__(**kwargs)

# Apply monkey patch to customtkinter globally
customtkinter.CTkFont = UnifiedCTkFont

left_jog_buttons = [0,0,0,0,0,0]
right_jog_buttons  =[0,0,0,0,0,0]
translation_buttons = [0,0,0,0,0,0]
rotation_buttons = [0,0,0,0,0,0]

left_frames_width = 430
right_frames_width = 300
bottom_frame_height = 96

# === Elegant Black Professional palette ===
# Pure black foundation with subtle warm-neutral surfaces. Refined, premium feel
# with restrained accent colors. Gold branding, crisp white typography.
UI_APP_BG          = "#000000"  # pure black — app background
UI_SURFACE         = "#0a0a0a"  # near-black panel face
UI_SURFACE_ALT     = "#111111"  # slightly lifted — header strips
UI_SURFACE_LOW     = "#080808"  # deepest recessed panels
UI_SURFACE_HIGH    = "#1a1a1a"  # elevated surfaces / data wells
UI_BORDER          = "#1f1f1f"  # subtle borders — low contrast
UI_OUTLINE         = "#2a2a2a"  # stronger separators
UI_HANDLE          = "#252525"  # drag handles
UI_ACCENT          = "#3366ff"  # refined blue accent
UI_ACCENT_DEEP     = "#5588ff"  # hover / pressed state
UI_ACCENT_SOFT     = "#0d1a33"  # subtle accent tint
UI_GOLD            = "#d4a843"  # refined Polman gold — muted elegance
UI_ON_SURFACE      = "#f0f0f0"  # crisp white text
UI_ON_SURFACE_MUTE = "#808080"  # muted secondary text
UI_SUCCESS         = "#2ecc71"  # refined green — Start/Run
UI_DANGER          = "#e74c3c"  # refined red — Stop/Emergency
UI_WARN            = "#f39c12"  # refined amber — Reset/Standby

class CollapsibleFrame(customtkinter.CTkFrame):
    def __init__(self, parent, title, content_height=None, start_collapsed=False, **kwargs):
        super().__init__(parent, corner_radius=8, border_width=1, border_color=UI_BORDER, fg_color=UI_SURFACE, **kwargs)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

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
            self.content_frame = customtkinter.CTkFrame(self, fg_color="transparent", corner_radius=0, height=content_height)
        else:
            self.content_frame = customtkinter.CTkFrame(self, fg_color="transparent", corner_radius=0)
            
        self.content_frame.grid(row=1, column=0, sticky="nsew", padx=6, pady=(0, 8))
        if content_height:
            self.content_frame.grid_propagate(False)
        self.content_frame.columnconfigure((0, 1, 2, 3), weight=1)
        
        self.is_collapsed = False
        self.header_frame.bind("<Button-1>", lambda event: self.toggle())
        self.title_label.bind("<Button-1>", lambda event: self.toggle())
        if start_collapsed:
            self.toggle()

    def toggle(self):
        if self.is_collapsed:
            self.rowconfigure(1, weight=1)
            self.content_frame.grid(row=1, column=0, sticky="nsew", padx=6, pady=(0, 8))
            self.toggle_btn.configure(text="▼")
            self.is_collapsed = False
        else:
            self.content_frame.grid_forget()
            self.rowconfigure(1, weight=0)
            self.toggle_btn.configure(text="▶")
            self.is_collapsed = True

prev_positions = np.array([0,0,0,0,0,0])
robot_pose = [0,0,0,0,0,0] #np.array([0,0,0,0,0,0])

padx_top_bot = 20
def GUI(shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons,program_log_queue=None):
    


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
            (getattr(app, "Vision_button", None), {"Vision"}),
            (getattr(app, "Modbus_button", None), {"Modbus"}),
            (getattr(app, "Research_button", None), {"Research"}),
        ]
        for btn, active_for in tabs:
            if btn is None:
                continue
            if active_name in active_for:
                btn.configure(fg_color=UI_ACCENT, hover_color=UI_ACCENT_DEEP, text_color="#ffffff", font=customtkinter.CTkFont(size=13, weight="bold"))
            else:
                btn.configure(fg_color="transparent", hover_color=UI_ACCENT_SOFT, text_color=UI_ON_SURFACE_MUTE, font=customtkinter.CTkFont(size=13, weight="normal"))

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
        app.menu_select_frame.grid_columnconfigure(0, weight=0)  # logo
        app.menu_select_frame.grid_columnconfigure(6, weight=1)  # spacer pushes fw_label + help right
        app.menu_select_frame.grid_rowconfigure(0, weight=0)

        # --- Polman Logo (left side of header bar) ---
        try:
            _logo_img = Image.open(os.path.join(Image_path, "logo_polman_icon.png"))
            app._header_logo_ctk = customtkinter.CTkImage(_logo_img, size=(32, 32))
            app._header_logo_label = customtkinter.CTkLabel(
                app.menu_select_frame, image=app._header_logo_ctk, text="",
                fg_color="transparent"
            )
            app._header_logo_label.grid(row=0, column=0, padx=(12, 4), pady=5, sticky="w")
        except Exception as e:
            logging.warning(f"Could not load header logo: {e}")

        # Tab glyph icons (Unicode, no asset files required).
        _tab_font = customtkinter.CTkFont(size=13, weight="bold")

        # Move button — compass / move arrows
        app.move_mode_select_button = customtkinter.CTkButton(app.menu_select_frame,text="✥  Move", font=_tab_font, command = raise_frame_jog)
        app.move_mode_select_button.grid(row=0, column=1, padx=(10,0),pady = 5,sticky="nw")

        # I/O button — bidirectional arrows
        app.I0_mode_select_button = customtkinter.CTkButton(app.menu_select_frame,text="⇅  I/O", font=_tab_font, command = raise_frame_IO)
        app.I0_mode_select_button.grid(row=0, column=2, padx=(10,0),pady = 5,sticky="nw")

        # Vision button — camera lens / eye
        app.Vision_button = customtkinter.CTkButton(app.menu_select_frame,text="◉  Vision", font=_tab_font, command = raise_vision_frame)
        app.Vision_button.grid(row=0, column=3, padx=(10,0) ,pady = 5,sticky="nw")

        # Modbus button — bolt / live link
        app.Modbus_button = customtkinter.CTkButton(app.menu_select_frame,text="⚡  Modbus", font=_tab_font, command = raise_modbus_frame)
        app.Modbus_button.grid(row=0, column=4, padx=(10,0) ,pady = 5,sticky="nw")

        # Research button — chart / analytics
        app.Research_button = customtkinter.CTkButton(app.menu_select_frame,text="▤  Research", font=_tab_font, command = raise_research_frame)
        app.Research_button.grid(row=0, column=5, padx=(10,0) ,pady = 5,sticky="nw")

        app.fw_label = customtkinter.CTkLabel(app.menu_select_frame, text="Source controller fw v1.0.0", text_color=UI_GOLD, font=customtkinter.CTkFont(family='JetBrains Mono', size=11, weight='bold'))
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
        # Add a helpful label for the current platform.
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

        joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']

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


    app.cart_frame = CollapsibleFrame(app.left_upper_stack, title="Cartesian Jog Controls", content_height=620)
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

    VISION_RUNTIME_OPTIONS = {
        "cuda": "GPU (CUDA)",
        "cpu": "CPU",
        "auto": "Auto",
    }

    def _vision_runtime_value(value):
        text = str(value or "auto").strip().lower()
        aliases = {
            "gpu (cuda)": "cuda",
            "gpu": "cuda",
            "cuda": "cuda",
            "nvidia": "cuda",
            "cpu": "cpu",
            "auto": "auto",
        }
        return aliases.get(text, "auto")

    def _vision_runtime_label(value):
        return VISION_RUNTIME_OPTIONS.get(_vision_runtime_value(value), VISION_RUNTIME_OPTIONS["auto"])

    # ------------------------------------------------------------------
    # Test Accuracy Pick Point (Test B) — self-contained, on/off via config.
    # Reuses vision/pick_accuracy_validator.py; never touches the live path.
    # ------------------------------------------------------------------
    def _pick_acc_make_validator():
        from vision.pick_accuracy_validator import PickAccuracyValidator
        return PickAccuracyValidator()

    def _pick_acc_safe_float(text, default=0.0):
        try:
            text = str(text).strip()
            return float(text) if text != "" else default
        except Exception:
            return default

    def _pick_acc_pair(seq):
        try:
            if seq is None:
                return None
            values = list(seq)
            if len(values) < 2:
                return None
            return (float(values[0]), float(values[1]))
        except Exception:
            return None

    def _pick_acc_parse_box(text):
        try:
            parts = [p for p in str(text).replace(";", ",").split(",") if p.strip() != ""]
            if len(parts) != 4:
                return None
            return [float(p) for p in parts]
        except Exception:
            return None

    def _pick_acc_calibration():
        cal = load_config().get("vision", {}).get("calibration", {})
        intr = cal.get("intrinsic_matrix")
        dist = cal.get("distortion")
        img = cal.get("image_size")
        intrinsic = np.array(intr, dtype=float) if intr else None
        distortion = np.array(dist, dtype=float) if dist else None
        frame_size = (int(img[0]), int(img[1])) if img and len(img) >= 2 else None
        return intrinsic, distortion, frame_size

    def _pick_acc_reference_distance(bundle):
        """Program distance (mm): selongsong tip -> pick point, from a live bundle."""
        sel_box = bundle.get("selongsong_box") if bundle else None
        pick_base = bundle.get("pick_point_base") if bundle else None
        if sel_box is None or pick_base is None:
            return None
        intrinsic, distortion, frame_size = _pick_acc_calibration()
        if intrinsic is None:
            return None
        try:
            return _pick_acc_make_validator().reference_distance(
                sel_box, pick_base, intrinsic, distortion, frame_size
            )
        except Exception:
            return None

    def pick_acc_is_enabled():
        return bool(
            load_config().get("vision", {}).get("pick_accuracy_test", {}).get("enabled", False)
        )

    def _pick_acc_refresh_enabled_ui():
        enabled = pick_acc_is_enabled()
        btn = getattr(app, "pick_acc_enable_btn", None)
        if btn is not None:
            btn.configure(
                text=f"Test: {'ON' if enabled else 'OFF'}",
                fg_color=UI_SUCCESS if enabled else UI_SURFACE_HIGH,
            )
        state = "normal" if enabled else "disabled"
        for widget in getattr(app, "pick_acc_widgets", []):
            try:
                widget.configure(state=state)
            except Exception:
                pass

    def _pick_acc_persist_threshold():
        cfg = load_config()
        block = cfg.setdefault("vision", {}).setdefault("pick_accuracy_test", {})
        entry = app.pick_acc_entries.get("Success threshold (mm)")
        if entry is not None:
            block["threshold_mm"] = _pick_acc_safe_float(entry[0].get(), block.get("threshold_mm", 3.0))
        save_config(cfg)

    def toggle_pick_accuracy_test():
        cfg = load_config()
        block = cfg.setdefault("vision", {}).setdefault("pick_accuracy_test", {})
        block["enabled"] = not bool(block.get("enabled", False))
        entry = app.pick_acc_entries.get("Success threshold (mm)")
        if entry is not None:
            block["threshold_mm"] = _pick_acc_safe_float(entry[0].get(), block.get("threshold_mm", 3.0))
        save_config(cfg)
        _pick_acc_refresh_enabled_ui()

    def _pick_acc_set_ref_label(distance):
        app._pick_acc_ref_distance = distance
        ref_label = getattr(app, "pick_acc_ref_label", None)
        if ref_label is not None:
            ref_label.configure(
                text=(
                    f"Program ujung→pick: {distance:.1f} mm"
                    if distance is not None
                    else "Program ujung→pick: -"
                )
            )

    def pick_acc_capture_detection():
        bundle = vision_manager.latest_bundle()
        label = getattr(app, "pick_acc_detect_label", None)
        if label is None:
            return bundle
        if not bundle:
            label.configure(text="Deteksi: (belum ada) — Start Camera + Detection ON dulu")
            _pick_acc_set_ref_label(None)
            return None
        status = str(bundle.get("pick_safety", "UNKNOWN"))
        px = _pick_acc_pair(bundle.get("pick_point_px"))
        base = _pick_acc_pair(bundle.get("pick_point_base"))
        px_s = f"({px[0]:.0f},{px[1]:.0f})px" if px else "-"
        base_s = f"({base[0]:.1f},{base[1]:.1f})mm" if base else "-"
        label.configure(text=f"Deteksi: {status} • {px_s} • {base_s}")
        _pick_acc_set_ref_label(_pick_acc_reference_distance(bundle))
        return bundle

    def pick_acc_add_trial():
        if not pick_acc_is_enabled():
            messagebox.showwarning(
                "Test Accuracy Pick Point",
                "Fitur masih OFF. Tekan tombol 'Test: OFF' supaya jadi ON dulu.",
            )
            return
        bundle = vision_manager.latest_bundle()
        if not bundle:
            messagebox.showwarning(
                "Test Accuracy Pick Point",
                "Belum ada deteksi. Start Camera, Detection ON, arahkan ke objek, lalu coba lagi.",
            )
            return
        _pick_acc_persist_threshold()
        entries = app.pick_acc_entries
        label = entries["Object label"][0].get().strip()
        orientation = _pick_acc_safe_float(entries["Orientation (deg)"][0].get(), 0.0)
        measured_raw = entries["Measured tip→tool (mm)"][0].get().strip()
        gt_sel = _pick_acc_parse_box(entries["GT selongsong box"][0].get())
        gt_fix = _pick_acc_parse_box(entries["GT fixture box"][0].get())
        measured_distance = _pick_acc_safe_float(measured_raw, None) if measured_raw != "" else None

        det_sel = bundle.get("selongsong_box")
        det_fix = bundle.get("fixture_box")
        intrinsic, distortion, frame_size = _pick_acc_calibration()

        trial_id = f"T{len(app._pick_acc_trials) + 1:03d}"
        try:
            validator = _pick_acc_make_validator()
            if det_sel is not None and intrinsic is not None:
                trial = validator.build_trial(
                    trial_id,
                    detected_selongsong_box=det_sel,
                    detected_fixture_box=det_fix,
                    ground_truth_selongsong_box=gt_sel,
                    ground_truth_fixture_box=gt_fix,
                    intrinsic_matrix=intrinsic,
                    distortion=distortion,
                    frame_size=frame_size,
                    measured_distance_mm=measured_distance,
                    object_label=label,
                    orientation_deg=orientation,
                )
            else:
                from vision.pick_accuracy_validator import PickTrial
                trial = PickTrial(
                    trial_id=trial_id,
                    object_label=label,
                    orientation_deg=orientation,
                    detected_pick_px=_pick_acc_pair(bundle.get("pick_point_px")),
                    detected_pick_base=_pick_acc_pair(bundle.get("pick_point_base")),
                    detected_status=str(bundle.get("pick_safety", "UNKNOWN")),
                    measured_distance_mm=measured_distance,
                )
        except RuntimeError as exc:
            messagebox.showwarning("Test Accuracy Pick Point", str(exc))
            return
        except Exception as exc:
            messagebox.showerror("Test Accuracy Pick Point", f"Gagal menambah trial: {exc}")
            return

        app._pick_acc_trials.append(trial)
        try:
            validator.save_trials(app._pick_acc_trials)
        except Exception:
            pass
        for key in ("Measured tip→tool (mm)", "GT selongsong box", "GT fixture box"):
            entries[key][0].delete(0, "end")
        pick_acc_capture_detection()
        pick_acc_show_report()

    def pick_acc_show_report():
        _pick_acc_persist_threshold()
        try:
            validator = _pick_acc_make_validator()
            report = validator.analyze(app._pick_acc_trials)
            text = validator.format_report(report)
        except Exception as exc:
            text = f"Report error: {exc}"
            report = None
        box = getattr(app, "pick_acc_report_box", None)
        if box is not None:
            box.configure(state="normal")
            box.delete("1.0", "end")
            box.insert("1.0", text)
            box.configure(state="disabled")
        status = getattr(app, "pick_acc_status", None)
        if status is not None and report is not None:
            status.configure(
                text=f"{report.trial_count} trial • diukur {report.measured_trial_count} "
                f"• sukses {report.success_count}/{report.measured_trial_count}"
            )

    def pick_acc_export_csv():
        if not app._pick_acc_trials:
            messagebox.showwarning("Export CSV", "Belum ada trial untuk diexport.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv")],
            initialfile="pick_accuracy_trials.csv",
        )
        if not path:
            return
        try:
            _pick_acc_make_validator().export_csv(app._pick_acc_trials, path)
        except Exception as exc:
            messagebox.showerror("Export CSV", f"Gagal export: {exc}")
            return
        messagebox.showinfo("Export CSV", f"Tersimpan ke:\n{path}")

    def pick_acc_load_log():
        try:
            loaded = _pick_acc_make_validator().load_trials()
        except Exception as exc:
            messagebox.showerror("Load Log", f"Gagal memuat log: {exc}")
            return
        app._pick_acc_trials = list(loaded)
        pick_acc_show_report()
        messagebox.showinfo("Load Log", f"Memuat {len(loaded)} trial dari log.")

    def pick_acc_clear_trials():
        if not app._pick_acc_trials:
            return
        if not messagebox.askyesno(
            "Clear Trials", f"Hapus {len(app._pick_acc_trials)} trial dari sesi ini?"
        ):
            return
        app._pick_acc_trials = []
        pick_acc_show_report()

    def vision_frame(host=None):
        host = host or app.vision_frame
        for child in host.winfo_children():
            child.destroy()
        host.grid_columnconfigure(0, weight=0, minsize=app.vision_left_width)
        host.grid_columnconfigure(1, weight=0)
        host.grid_columnconfigure(2, weight=1)
        host.grid_rowconfigure(1, weight=1)
        app.vision_host = host

        cfg_all = load_config()
        cfg = cfg_all["vision"]
        workspace = cfg_all["workspace"]
        object_info_cache = getattr(app, "_vision_object_info_cache", {})
        cached_info_labels = (
            object_info_cache.get("labels", {})
            if isinstance(object_info_cache, dict)
            else {}
        )
        cached_safety = (
            str(object_info_cache.get("safety", "UNKNOWN")).upper()
            if isinstance(object_info_cache, dict)
            else "UNKNOWN"
        )

        title = customtkinter.CTkLabel(host, text="Vision System", font=customtkinter.CTkFont(size=20, weight="bold"))
        title.grid(row=0, column=0, columnspan=2, padx=16, pady=(12, 6), sticky="w")
        detached = host is not app.vision_frame
        app.vision_dock_button = customtkinter.CTkButton(
            host,
            text="Dock Vision" if detached else "Detach Vision",
            width=110,
            command=dock_vision_window if detached else detach_vision_window,
        )
        app.vision_dock_button.grid(row=0, column=2, padx=16, pady=(12, 6), sticky="e")

        live_panel = customtkinter.CTkFrame(host, corner_radius=0)
        live_panel.grid(row=1, column=0, padx=(12, 2), pady=(0, 12), sticky="nsew")
        live_panel.grid_columnconfigure(0, weight=1)
        live_panel.grid_rowconfigure(1, weight=1)

        # Draggable Divider Handle
        app.vision_handle = customtkinter.CTkFrame(host, width=7, corner_radius=3, fg_color=UI_HANDLE, cursor="sb_h_double_arrow")
        app.vision_handle.grid(row=1, column=1, sticky="ns", padx=2, pady=12)

        def start_vision_resize(event):
            app.vision_drag_start_x = event.x_root
            app.vision_drag_start_width = app.vision_left_width

        def drag_vision_resize(event):
            delta = event.x_root - app.vision_drag_start_x
            new_width = max(300, min(1400, app.vision_drag_start_width + delta))
            app.vision_left_width = new_width
            host.grid_columnconfigure(0, minsize=new_width)

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
        app.vision_detection_on = customtkinter.CTkButton(
            controls,
            text="Detection ON",
            width=100,
            command=lambda: set_vision_detection(True),
        )
        app.vision_detection_on.grid(row=1, column=3, padx=6, pady=4)
        app.vision_detection_off = customtkinter.CTkButton(
            controls,
            text="Detection OFF",
            width=100,
            command=lambda: set_vision_detection(False),
        )
        app.vision_detection_off.grid(row=1, column=4, padx=6, pady=4)

        app.vision_feed_label = customtkinter.CTkLabel(live_panel, text="Camera feed not started", anchor="center")
        app.vision_feed_label.grid(row=1, column=0, padx=8, pady=8, sticky="nsew")
        app.vision_feed_label.bind("<Button-1>", capture_camera_base_pixel)
        app.vision_safety_badge = customtkinter.CTkLabel(
            live_panel,
            text=cached_safety,
            height=28,
            anchor="center",
        )
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

        side = customtkinter.CTkScrollableFrame(host, corner_radius=0)
        side.grid(row=1, column=2, padx=(2, 12), pady=(0, 12), sticky="nsew")
        side.grid_columnconfigure(0, weight=1)
        app.vision_entries = {}

        detection = CollapsibleFrame(side, title="Detection Setup")
        detection.grid(row=0, column=0, padx=4, pady=6, sticky="ew")
        detection.content_frame.grid_columnconfigure((1, 3), weight=1)
        customtkinter.CTkLabel(
            detection.content_frame,
            text="Method: ONNX model",
            anchor="w",
        ).grid(row=1, column=0, columnspan=4, padx=6, pady=4, sticky="we")
        _grid_labeled_entry(detection.content_frame, 2, 0, "Brightness", cfg.get("brightness", 0), store=app.vision_entries, cast=int)
        _grid_labeled_entry(detection.content_frame, 2, 2, "Contrast", cfg.get("contrast", 1.0), store=app.vision_entries, cast=float)
        _grid_labeled_entry(detection.content_frame, 3, 0, "Zoom", cfg.get("zoom", 1.0), store=app.vision_entries, cast=float)
        _grid_labeled_entry(detection.content_frame, 3, 2, "Camera width", cfg.get("camera_width", 1280), store=app.vision_entries, cast=int)
        _grid_labeled_entry(detection.content_frame, 4, 0, "Camera height", cfg.get("camera_height", 720), store=app.vision_entries, cast=int)
        app.vision_apply = customtkinter.CTkButton(
            detection.content_frame,
            text="Apply Detection",
            command=save_vision_settings,
        )
        app.vision_apply.grid(row=5, column=0, columnspan=4, padx=6, pady=8, sticky="we")

        model = CollapsibleFrame(side, title="Model Config")
        model.grid(row=1, column=0, padx=4, pady=6, sticky="ew")
        model.content_frame.grid_columnconfigure(1, weight=1)
        customtkinter.CTkLabel(model.content_frame, text="ONNX path").grid(row=1, column=0, padx=6, pady=4, sticky="w")
        app.vision_model_entry = _entry(model.content_frame, cfg.get("model_path", "vision/models/best.onnx"), width=300)
        app.vision_model_entry.grid(row=1, column=1, padx=6, pady=4, sticky="we")
        customtkinter.CTkButton(model.content_frame, text="Browse", width=70, command=browse_vision_model).grid(row=1, column=2, padx=6, pady=4)
        customtkinter.CTkButton(model.content_frame, text="Load", width=70, command=load_vision_model).grid(row=1, column=3, padx=6, pady=4)
        customtkinter.CTkLabel(model.content_frame, text="Runtime").grid(row=2, column=0, padx=6, pady=4, sticky="w")
        app.vision_model_runtime_menu = customtkinter.CTkOptionMenu(
            model.content_frame,
            values=list(VISION_RUNTIME_OPTIONS.values()),
        )
        app.vision_model_runtime_menu.set(_vision_runtime_label(cfg.get("model_runtime", "auto")))
        app.vision_model_runtime_menu.grid(row=2, column=1, columnspan=3, padx=6, pady=4, sticky="we")
        _grid_labeled_entry(model.content_frame, 3, 0, "Conf", cfg.get("model_conf_threshold", 0.5), store=app.vision_entries, cast=float)
        _grid_labeled_entry(model.content_frame, 3, 2, "IoU", cfg.get("model_iou_threshold", 0.45), store=app.vision_entries, cast=float)
        app.vision_model_status = customtkinter.CTkLabel(model.content_frame, text="Model: NOT LOADED", anchor="w")
        app.vision_model_status.grid(row=4, column=0, columnspan=4, padx=6, pady=4, sticky="we")

        info = CollapsibleFrame(side, title="Object Info")
        info.grid(row=2, column=0, padx=4, pady=6, sticky="ew")
        info.content_frame.grid_columnconfigure(1, weight=1)
        app.vision_info_labels = {}
        for idx, key in enumerate(["Status", "BBox", "Centroid", "Dimension", "Pick point", "Orientation"]):
            customtkinter.CTkLabel(info.content_frame, text=key).grid(row=idx + 1, column=0, padx=6, pady=3, sticky="w")
            label = customtkinter.CTkLabel(
                info.content_frame,
                text=str(cached_info_labels.get(key, "-")),
                anchor="w",
                justify="left",
            )
            label.grid(row=idx + 1, column=1, padx=6, pady=3, sticky="we")
            app.vision_info_labels[key] = label

        pick_zone = CollapsibleFrame(side, title="Safe Pick Zone")
        pick_zone.grid(row=3, column=0, padx=4, pady=6, sticky="ew")
        pick_zone.content_frame.grid_columnconfigure((1, 3), weight=1)
        _grid_labeled_entry(pick_zone.content_frame, 1, 0, "Safe margin", cfg.get("safe_pick_margin_pct", 0.25), store=app.vision_entries, cast=float)
        _grid_labeled_entry(pick_zone.content_frame, 1, 2, "Left offset px", cfg.get("safe_pick_left_offset_px", 0.0), store=app.vision_entries, cast=float)
        _grid_labeled_entry(pick_zone.content_frame, 2, 0, "Marginal timeout", cfg.get("marginal_confirm_timeout_s", 2.0), store=app.vision_entries, cast=float)

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
        _grid_labeled_entry(workspace_panel.content_frame, 5, 0, "Z Tool offset", cfg.get("z_tool_offset_mm", 0), store=app.vision_entries, cast=float)

        vision_tool = CollapsibleFrame(side, title="Vision Tool")
        vision_tool.grid(row=5, column=0, padx=4, pady=6, sticky="ew")
        vision_tool.content_frame.grid_columnconfigure((1, 3), weight=1)
        tool_cfg = cfg.get("tool_z", {})
        _grid_labeled_entry(vision_tool.content_frame, 1, 0, "Reference joints", ",".join(str(x) for x in tool_cfg.get("reference_joint_deg", [90, -88, 182.259, 0, 3, 180])), width=180, store=app.vision_entries)
        _grid_labeled_entry(vision_tool.content_frame, 1, 2, "Pose tolerance", tool_cfg.get("pose_tolerance_deg", 1.0), store=app.vision_entries, cast=float)
        _grid_labeled_entry(vision_tool.content_frame, 2, 0, "X Tool fixed", tool_cfg.get("x_tool_fixed_mm", -60.0), store=app.vision_entries, cast=float)
        _grid_labeled_entry(vision_tool.content_frame, 2, 2, "Y Tool fixed", tool_cfg.get("y_tool_fixed_mm", 0.0), store=app.vision_entries, cast=float)
        _grid_labeled_entry(vision_tool.content_frame, 3, 0, "Z+ min", tool_cfg.get("z_plus_min_mm", 0.0), store=app.vision_entries, cast=float)
        _grid_labeled_entry(vision_tool.content_frame, 3, 2, "Z+ max", tool_cfg.get("z_plus_max_mm", 78.0), store=app.vision_entries, cast=float)
        _grid_labeled_entry(vision_tool.content_frame, 4, 0, "Retreat margin", tool_cfg.get("retreat_margin_mm", 30.0), store=app.vision_entries, cast=float)
        _grid_labeled_entry(vision_tool.content_frame, 4, 2, "Vision samples", tool_cfg.get("sample_count", 5), store=app.vision_entries, cast=int)
        _grid_labeled_entry(vision_tool.content_frame, 5, 0, "Detection age", tool_cfg.get("detection_max_age_s", 1.0), store=app.vision_entries, cast=float)
        _grid_labeled_entry(vision_tool.content_frame, 5, 2, "Runtime age", tool_cfg.get("runtime_max_age_s", 30.0), store=app.vision_entries, cast=float)
        customtkinter.CTkButton(
            vision_tool.content_frame,
            text="Test Compute Tool-Z (Optional)",
            command=run_vision_pick_now,
        ).grid(row=6, column=0, columnspan=4, padx=6, pady=(8, 4), sticky="we")

        calibration = CollapsibleFrame(side, title="Camera Calibration")
        calibration.grid(row=6, column=0, padx=4, pady=6, sticky="ew")
        calibration.content_frame.grid_columnconfigure((1, 3), weight=1)
        cal_cfg = cfg.get("calibration", {})
        camera_base_points = list(getattr(app, "camera_base_points", []))
        if not camera_base_points:
            camera_base_points = load_camera_base_points_from_config()
        camera_base_pending_pixel = getattr(app, "camera_base_pending_pixel", None)
        camera_base_calibration_active = bool(
            getattr(app, "camera_base_calibration_active", False)
        )
        camera_base_points_visible = bool(
            getattr(
                app,
                "camera_base_points_visible",
                cfg.get("camera_to_base_points_visible", True),
            )
        )
        camera_base_selected_index = getattr(app, "camera_base_selected_index", None)
        if (
            not isinstance(camera_base_selected_index, int)
            or camera_base_selected_index < 0
            or camera_base_selected_index >= len(camera_base_points)
        ):
            camera_base_selected_index = None
        app.camera_base_calibration_active = camera_base_calibration_active
        app.camera_base_points_visible = camera_base_points_visible
        app.camera_base_selected_index = camera_base_selected_index
        app.camera_calibration_toggle = customtkinter.CTkButton(
            calibration.content_frame,
            text=f"Calibration Click: {'ON' if camera_base_calibration_active else 'OFF'}",
            fg_color=UI_SUCCESS if camera_base_calibration_active else UI_SURFACE_HIGH,
            command=toggle_camera_base_calibration,
        )
        app.camera_calibration_toggle.grid(
            row=0,
            column=0,
            columnspan=4,
            padx=6,
            pady=(6, 2),
            sticky="we",
        )
        _grid_labeled_entry(calibration.content_frame, 1, 0, "Chessboard", ",".join(str(x) for x in cal_cfg.get("chessboard_size", [9, 6])), width=120, store=app.vision_entries)
        _grid_labeled_entry(calibration.content_frame, 1, 2, "Square mm", cal_cfg.get("square_size_mm", 25.0), store=app.vision_entries, cast=float)
        customtkinter.CTkButton(calibration.content_frame, text="Snap Frame", command=capture_vision_snapshot).grid(row=2, column=0, padx=6, pady=6, sticky="we")
        customtkinter.CTkButton(calibration.content_frame, text="Run Calibration", command=run_vision_calibration).grid(row=2, column=1, padx=6, pady=6, sticky="we")
        customtkinter.CTkButton(
            calibration.content_frame,
            text="Reset Intrinsic",
            fg_color=UI_DANGER,
            hover_color="#9F1239",
            command=reset_vision_intrinsic,
        ).grid(row=2, column=2, columnspan=2, padx=6, pady=6, sticky="we")
        app.vision_chessboard_preview = customtkinter.CTkCheckBox(
            calibration.content_frame,
            text="Show chessboard corners",
            command=save_vision_settings,
        )
        app.vision_chessboard_preview.grid(
            row=3,
            column=0,
            columnspan=4,
            padx=6,
            pady=6,
            sticky="w",
        )
        if bool(cal_cfg.get("preview_enabled", False)) or camera_base_calibration_active:
            app.vision_chessboard_preview.select()
        app.camera_base_points_visible_checkbox = customtkinter.CTkCheckBox(
            calibration.content_frame,
            text="Show Camera-to-Base points",
            command=toggle_camera_base_points_visibility,
        )
        app.camera_base_points_visible_checkbox.grid(
            row=4,
            column=0,
            columnspan=4,
            padx=6,
            pady=(0, 6),
            sticky="w",
        )
        if camera_base_points_visible:
            app.camera_base_points_visible_checkbox.select()
        app.vision_calibration = customtkinter.CTkLabel(calibration.content_frame, text="Calibration: -", anchor="w", justify="left")
        app.vision_calibration.grid(row=5, column=0, columnspan=4, padx=6, pady=4, sticky="we")
        app.camera_base_points = camera_base_points
        app.camera_base_pending_pixel = camera_base_pending_pixel
        app.camera_base_point_status = customtkinter.CTkLabel(
            calibration.content_frame,
            text=(
                f"Calibration click {'ON' if camera_base_calibration_active else 'OFF'}; "
                f"{len(camera_base_points)}/9 points saved"
            ),
            anchor="w",
            justify="left",
        )
        app.camera_base_point_status.grid(row=6, column=0, columnspan=4, padx=6, pady=4, sticky="we")
        customtkinter.CTkButton(calibration.content_frame, text="Add / Overwrite TCP", command=add_camera_base_point).grid(row=7, column=0, padx=6, pady=6, sticky="we")
        customtkinter.CTkButton(calibration.content_frame, text="Delete Selected", command=delete_selected_camera_base_point).grid(row=7, column=1, padx=6, pady=6, sticky="we")
        customtkinter.CTkButton(calibration.content_frame, text="Reset 9 Points", command=reset_camera_base_points).grid(row=7, column=2, padx=6, pady=6, sticky="we")
        customtkinter.CTkButton(calibration.content_frame, text="Solve Camera-to-Base", command=solve_camera_to_base).grid(row=7, column=3, padx=6, pady=6, sticky="we")
        app.camera_base_points_text = customtkinter.CTkTextbox(calibration.content_frame, height=120)
        app.camera_base_points_text.grid(row=8, column=0, columnspan=4, padx=6, pady=6, sticky="we")
        app.camera_base_points_text.bind("<ButtonRelease-1>", select_camera_base_point_from_text)
        refresh_camera_base_points()

        # ---- Test Accuracy Pick Point (Test B) ----
        pa_cfg = cfg.get("pick_accuracy_test", {})
        if not hasattr(app, "_pick_acc_trials"):
            app._pick_acc_trials = []
        pick_acc = CollapsibleFrame(side, title="Test Accuracy Pick Point")
        pick_acc.grid(row=7, column=0, padx=4, pady=6, sticky="ew")
        pa = pick_acc.content_frame
        pa.grid_columnconfigure((1, 3), weight=1)
        app.pick_acc_entries = {}

        pa_enabled = bool(pa_cfg.get("enabled", False))
        app.pick_acc_enable_btn = customtkinter.CTkButton(
            pa,
            text=f"Test: {'ON' if pa_enabled else 'OFF'}",
            fg_color=UI_SUCCESS if pa_enabled else UI_SURFACE_HIGH,
            command=toggle_pick_accuracy_test,
        )
        app.pick_acc_enable_btn.grid(row=0, column=0, padx=6, pady=(6, 4), sticky="we")
        _grid_labeled_entry(pa, 0, 2, "Success threshold (mm)", pa_cfg.get("threshold_mm", 3.0), store=app.pick_acc_entries, cast=float)

        app.pick_acc_detect_label = customtkinter.CTkLabel(pa, text="Deteksi: (tekan Refresh)", anchor="w", justify="left")
        app.pick_acc_detect_label.grid(row=1, column=0, columnspan=3, padx=6, pady=4, sticky="we")
        pa_refresh = customtkinter.CTkButton(pa, text="Refresh", width=70, command=pick_acc_capture_detection)
        pa_refresh.grid(row=1, column=3, padx=6, pady=4, sticky="we")

        app.pick_acc_ref_label = customtkinter.CTkLabel(pa, text="Program ujung→pick: -", anchor="w", justify="left")
        app.pick_acc_ref_label.grid(row=2, column=0, columnspan=4, padx=6, pady=(0, 4), sticky="we")

        _grid_labeled_entry(pa, 3, 0, "Object label", "", store=app.pick_acc_entries)
        _grid_labeled_entry(pa, 3, 2, "Orientation (deg)", "0", store=app.pick_acc_entries, cast=float)
        _grid_labeled_entry(pa, 4, 0, "Measured tip→tool (mm)", "", store=app.pick_acc_entries, cast=float)
        _grid_labeled_entry(pa, 5, 0, "GT selongsong box", "", width=150, store=app.pick_acc_entries)
        _grid_labeled_entry(pa, 5, 2, "GT fixture box", "", width=150, store=app.pick_acc_entries)
        customtkinter.CTkLabel(
            pa,
            text=("Wajib (B2): ukur caliper ujung-1 selongsong → ujung tool, ketik di 'Measured tip→tool'.\n"
                  "Program hitung ujung-1 → pick point; error = |measured − program|.\n"
                  "Opsional (B1): GT box x1,y1,x2,y2 = error seleksi deteksi."),
            anchor="w",
            justify="left",
            font=customtkinter.CTkFont(size=10),
        ).grid(row=6, column=0, columnspan=4, padx=6, pady=(0, 4), sticky="we")

        pa_add = customtkinter.CTkButton(pa, text="Add Trial", command=pick_acc_add_trial)
        pa_add.grid(row=7, column=0, padx=6, pady=6, sticky="we")
        pa_report = customtkinter.CTkButton(pa, text="Show Report", command=pick_acc_show_report)
        pa_report.grid(row=7, column=1, padx=6, pady=6, sticky="we")
        pa_export = customtkinter.CTkButton(pa, text="Export CSV", command=pick_acc_export_csv)
        pa_export.grid(row=7, column=2, padx=6, pady=6, sticky="we")
        pa_load = customtkinter.CTkButton(pa, text="Load Log", command=pick_acc_load_log)
        pa_load.grid(row=7, column=3, padx=6, pady=6, sticky="we")

        pa_clear = customtkinter.CTkButton(pa, text="Clear Trials", fg_color=UI_DANGER, hover_color="#9F1239", command=pick_acc_clear_trials)
        pa_clear.grid(row=8, column=0, padx=6, pady=(0, 6), sticky="we")
        app.pick_acc_status = customtkinter.CTkLabel(pa, text="0 trial", anchor="w")
        app.pick_acc_status.grid(row=8, column=1, columnspan=3, padx=6, pady=(0, 6), sticky="we")

        app.pick_acc_report_box = customtkinter.CTkTextbox(pa, height=210)
        app.pick_acc_report_box.grid(row=9, column=0, columnspan=4, padx=6, pady=6, sticky="we")
        app.pick_acc_report_box.configure(state="disabled")

        app.pick_acc_widgets = [
            pa_refresh, pa_add, pa_report, pa_export, pa_load, pa_clear,
            app.pick_acc_entries["Object label"][0],
            app.pick_acc_entries["Orientation (deg)"][0],
            app.pick_acc_entries["Measured tip→tool (mm)"][0],
            app.pick_acc_entries["GT selongsong box"][0],
            app.pick_acc_entries["GT fixture box"][0],
        ]
        _pick_acc_refresh_enabled_ui()
        if app._pick_acc_trials:
            pick_acc_show_report()

        app.vision_status = customtkinter.CTkLabel(side, text="Status: stopped", anchor="w", justify="left")
        app.vision_status.grid(row=8, column=0, padx=6, pady=8, sticky="we")
        app._last_vision_image_update = 0
        start_vision_feed_loop(app.vision_feed_label)
        start_vision_state_loop(app.vision_status)

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
        tests.grid_rowconfigure(0, weight=0)
        tests.grid_rowconfigure(1, weight=1)

        block_a_cfg = cfg.get("block_a", {})
        block_a = CollapsibleFrame(tests, title="Blok A - Protocol Performance Test", start_collapsed=True)
        block_a.grid(row=0, column=0, columnspan=2, padx=0, pady=(0, 6), sticky="ew")
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

        block_a.content_frame.grid_rowconfigure(6, weight=1)
        block_a_log_columns = ("n", "timestamp", "response_ms", "status", "detail")
        block_a_log_container = customtkinter.CTkFrame(block_a.content_frame, fg_color="transparent")
        block_a_log_container.grid(row=6, column=0, columnspan=10, padx=5, pady=(4, 2), sticky="nsew")
        block_a_log_container.grid_columnconfigure(0, weight=1)
        block_a_log_container.grid_rowconfigure(0, weight=1)
        app.modbus_block_a_log_tree = ttk.Treeview(block_a_log_container, columns=block_a_log_columns, show="headings", height=6)
        block_a_log_headings = {"n": "N", "timestamp": "Time", "response_ms": "RT (ms)", "status": "Status", "detail": "Detail"}
        block_a_log_widths = {"n": 50, "timestamp": 110, "response_ms": 80, "status": 70, "detail": 220}
        for column in block_a_log_columns:
            app.modbus_block_a_log_tree.heading(column, text=block_a_log_headings[column])
            app.modbus_block_a_log_tree.column(column, width=block_a_log_widths[column], stretch=column == "detail")
        app.modbus_block_a_log_tree.grid(row=0, column=0, sticky="nsew")
        block_a_log_scroll = ttk.Scrollbar(block_a_log_container, orient="vertical", command=app.modbus_block_a_log_tree.yview)
        block_a_log_scroll.grid(row=0, column=1, sticky="ns")
        app.modbus_block_a_log_tree.configure(yscrollcommand=block_a_log_scroll.set)
        customtkinter.CTkButton(block_a.content_frame, text="Export XLSX", width=110, command=export_modbus_block_a_xlsx).grid(row=7, column=0, columnspan=2, padx=5, pady=(2, 6), sticky="we")

        block_b_cfg = cfg.get("block_b", {})
        block_b = CollapsibleFrame(tests, title="Blok B - Cycle Time Test")
        block_b.grid(row=1, column=0, columnspan=2, padx=0, pady=0, sticky="nsew")
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

        block_b.content_frame.grid_rowconfigure(5, weight=1)
        block_b_log_columns = ("index", "timestamp", "status", "duration_s", "note")
        block_b_log_container = customtkinter.CTkFrame(block_b.content_frame, fg_color="transparent")
        block_b_log_container.grid(row=5, column=0, columnspan=10, padx=5, pady=(4, 2), sticky="nsew")
        block_b_log_container.grid_columnconfigure(0, weight=1)
        block_b_log_container.grid_rowconfigure(0, weight=1)
        app.modbus_block_b_log_tree = ttk.Treeview(block_b_log_container, columns=block_b_log_columns, show="headings", height=6)
        block_b_log_headings = {"index": "Idx", "timestamp": "Time", "status": "Status", "duration_s": "Duration (s)", "note": "Note"}
        block_b_log_widths = {"index": 50, "timestamp": 110, "status": 70, "duration_s": 100, "note": 220}
        for column in block_b_log_columns:
            app.modbus_block_b_log_tree.heading(column, text=block_b_log_headings[column])
            app.modbus_block_b_log_tree.column(column, width=block_b_log_widths[column], stretch=column == "note")
        app.modbus_block_b_log_tree.grid(row=0, column=0, sticky="nsew")
        block_b_log_scroll = ttk.Scrollbar(block_b_log_container, orient="vertical", command=app.modbus_block_b_log_tree.yview)
        block_b_log_scroll.grid(row=0, column=1, sticky="ns")
        app.modbus_block_b_log_tree.configure(yscrollcommand=block_b_log_scroll.set)
        customtkinter.CTkButton(block_b.content_frame, text="Export XLSX", width=110, command=export_modbus_block_b_xlsx).grid(row=6, column=0, columnspan=2, padx=5, pady=(2, 6), sticky="we")
        customtkinter.CTkButton(block_b.content_frame, text="Export Cycle Log", width=130, command=export_modbus_cycle_log_xlsx).grid(row=6, column=2, columnspan=2, padx=5, pady=(2, 6), sticky="we")
        customtkinter.CTkButton(block_b.content_frame, text="Clear Cycle Log", width=120, command=clear_modbus_cycle_log).grid(row=6, column=4, columnspan=2, padx=5, pady=(2, 6), sticky="we")

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
        widths = {"name": 140, "type": 90, "address": 90, "rw": 80, "desc": 280}
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
        runtime_menu = getattr(app, "vision_model_runtime_menu", None)
        runtime_value = _vision_runtime_value(
            runtime_menu.get()
            if runtime_menu is not None
            else load_config().get("vision", {}).get("model_runtime", "auto")
        )
        settings = {
            "video_source": app.vision_source_menu.get(),
            "detection_enabled": vision_manager.detection_enabled(),
            "detection_method": "model",
            "model_path": app.vision_model_entry.get().strip(),
            "model_runtime": runtime_value,
        }
        key_map = {
            "Brightness": "brightness",
            "Contrast": "contrast",
            "Zoom": "zoom",
            "Camera width": "camera_width",
            "Camera height": "camera_height",
            "Conf": "model_conf_threshold",
            "IoU": "model_iou_threshold",
            "Safe margin": "safe_pick_margin_pct",
            "Left offset px": "safe_pick_left_offset_px",
            "Marginal timeout": "marginal_confirm_timeout_s",
            "Offset X": "offset_x_mm",
            "Offset Y": "offset_y_mm",
            "Z Tool offset": "z_tool_offset_mm",
            "Square mm": "calibration.square_size_mm",
            "Pose tolerance": "tool_z.pose_tolerance_deg",
            "X Tool fixed": "tool_z.x_tool_fixed_mm",
            "Y Tool fixed": "tool_z.y_tool_fixed_mm",
            "Z+ min": "tool_z.z_plus_min_mm",
            "Z+ max": "tool_z.z_plus_max_mm",
            "Retreat margin": "tool_z.retreat_margin_mm",
            "Vision samples": "tool_z.sample_count",
            "Detection age": "tool_z.detection_max_age_s",
            "Runtime age": "tool_z.runtime_max_age_s",
        }
        for label, key in key_map.items():
            if label in entries:
                entry, cast = entries[label]
                current_vision = load_config()["vision"]
                if key.startswith("tool_z."):
                    nested_key = key.split(".", 1)[1]
                    default_value = current_vision.get("tool_z", {}).get(nested_key, "")
                else:
                    default_value = current_vision.get(key.split(".")[-1], "")
                value = _entry_value(entry, default_value, cast)
                if key.startswith("calibration."):
                    settings.setdefault("calibration", {})[key.split(".", 1)[1]] = value
                elif key.startswith("tool_z."):
                    settings.setdefault("tool_z", {})[key.split(".", 1)[1]] = value
                else:
                    settings[key] = value
        settings.setdefault("calibration", {})["chessboard_size"] = _parse_int_list(
            entries["Chessboard"][0].get(),
            [9, 6],
        )[:2]
        settings["calibration"]["preview_enabled"] = bool(
            getattr(app, "vision_chessboard_preview", None)
            and app.vision_chessboard_preview.get()
        )
        points_visible_checkbox = getattr(app, "camera_base_points_visible_checkbox", None)
        settings["camera_to_base_points_visible"] = (
            bool(points_visible_checkbox.get())
            if points_visible_checkbox is not None
            else bool(
                getattr(
                    app,
                    "camera_base_points_visible",
                    load_config().get("vision", {}).get(
                        "camera_to_base_points_visible",
                        True,
                    ),
                )
            )
        )
        settings.setdefault("tool_z", {})["reference_joint_deg"] = _parse_float_list(
            entries["Reference joints"][0].get(),
            [90.0, -88.0, 182.259, 0.0, 3.0, 180.0],
        )[:6]
        return settings

    def build_workspace_settings():
        workspace = {}
        key_map = {"X min": "x_min_mm", "X max": "x_max_mm", "Y min": "y_min_mm", "Y max": "y_max_mm", "Z fixed": "z_fixed_mm", "Margin": "margin_mm"}
        for label, key in key_map.items():
            entry, cast = app.workspace_entries[label]
            workspace[key] = _entry_value(entry, load_config()["workspace"].get(key, 0), cast)
        return workspace

    def save_vision_settings():
        settings = build_vision_settings()
        workspace = build_workspace_settings()
        previous_cfg = load_config()
        previous_vision = previous_cfg.get("vision", {})
        calibration_changed = (
            str(previous_vision.get("video_source", "")) != str(settings.get("video_source", ""))
            or float(previous_vision.get("zoom", 1.0)) != float(settings.get("zoom", 1.0))
            or int(previous_vision.get("camera_width", 1280)) != int(settings.get("camera_width", 1280))
            or int(previous_vision.get("camera_height", 720)) != int(settings.get("camera_height", 720))
        )
        cfg = previous_cfg
        calibration_update = settings.pop("calibration", {})
        cfg["vision"].update(settings)
        cfg["vision"].setdefault("calibration", {}).update(calibration_update)
        if calibration_changed:
            cfg["vision"].setdefault("camera_to_base", {})["valid"] = False
        cfg["workspace"].update(workspace)
        save_config(cfg)
        vision_manager.save_settings(settings, workspace)
        research_logger.record("vision_settings_saved", 1, "apply")
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
        save_vision_settings()
        status = vision_manager.load_model(app.vision_model_entry.get().strip())
        app.vision_model_status.configure(text="Model: " + str(status))
        research_logger.record("vision_model_load", 1 if str(status).startswith("LOADED") else 0, status)

    def start_vision_camera():
        save_vision_settings()
        source = app.vision_source_menu.get()
        vision_manager.start(source)
        research_logger.record("vision_start", 1, source)

    def stop_vision_camera():
        vision_manager.stop()
        research_logger.record("vision_stop", 1)

    def set_vision_detection(enabled):
        vision_manager.set_detection_enabled(enabled)
        app._last_vision_image_update = 0
        research_logger.record("vision_detection_enabled", 1 if enabled else 0)

    def vision_window_is_detached():
        window = getattr(app, "vision_detached_window", None)
        if window is None:
            return False
        try:
            return bool(window.winfo_exists())
        except Exception:
            return False

    def detach_vision_window():
        if vision_window_is_detached():
            app.vision_detached_window.lift()
            app.vision_detached_window.focus_force()
            return
        save_vision_settings()
        for child in app.vision_frame.winfo_children():
            child.destroy()

        placeholder = customtkinter.CTkFrame(app.vision_frame, fg_color="transparent")
        placeholder.grid(row=0, column=0, columnspan=3, rowspan=2, sticky="nsew")
        placeholder.grid_columnconfigure(0, weight=1)
        placeholder.grid_rowconfigure(0, weight=1)
        customtkinter.CTkLabel(
            placeholder,
            text="Vision is open in a separate window",
            font=customtkinter.CTkFont(size=18, weight="bold"),
        ).grid(row=0, column=0, padx=20, pady=(20, 8))
        customtkinter.CTkButton(
            placeholder,
            text="Dock Vision",
            command=dock_vision_window,
        ).grid(row=1, column=0, padx=20, pady=(0, 20))

        window = customtkinter.CTkToplevel(app)
        app.vision_detached_window = window
        window.title("PAROL6 Vision")
        window.geometry("1600x900")
        window.minsize(1000, 650)
        window.protocol("WM_DELETE_WINDOW", dock_vision_window)
        detached_host = customtkinter.CTkFrame(
            window,
            corner_radius=8,
            fg_color=UI_SURFACE,
            border_width=1,
            border_color=UI_BORDER,
        )
        detached_host.pack(fill="both", expand=True, padx=5, pady=5)
        app.vision_detached_host = detached_host
        vision_frame(detached_host)
        window.lift()

    def dock_vision_window():
        if not vision_window_is_detached():
            return
        try:
            save_vision_settings()
        except Exception as exc:
            logging.warning("Could not save Vision settings before docking: %s", exc)
        window = app.vision_detached_window
        app.vision_detached_window = None
        try:
            window.destroy()
        except Exception:
            pass
        vision_frame(app.vision_frame)
        if getattr(app, "current_menu", "") == "Vision":
            app.vision_frame.grid(
                row=1,
                column=0,
                columnspan=4,
                rowspan=4,
                padx=(5, 5),
                pady=5,
                sticky="news",
            )
            app.vision_frame.tkraise()

    def set_marginal_decision(decision):
        state = read_state("vision_confirmation", {})
        token = state.get("token", time.time()) if isinstance(state, dict) else time.time()
        update_state("vision_confirmation", {"status": decision, "token": token, "decision": decision, "updated_at": time.time()})
        research_logger.record("marginal_" + decision, 1)

    def set_chessboard_preview_for_camera_base_calibration(enabled):
        checkbox = getattr(app, "vision_chessboard_preview", None)
        if checkbox is not None:
            try:
                if enabled:
                    checkbox.select()
                else:
                    checkbox.deselect()
            except (tk.TclError, RuntimeError):
                pass

        cfg = load_config()
        vision_cfg = cfg.setdefault("vision", {})
        calibration_cfg = vision_cfg.get("calibration", {})
        if not isinstance(calibration_cfg, dict):
            calibration_cfg = {}
            vision_cfg["calibration"] = calibration_cfg
        if bool(calibration_cfg.get("preview_enabled", False)) != bool(enabled):
            calibration_cfg["preview_enabled"] = bool(enabled)
            save_config(cfg)

    def toggle_camera_base_calibration():
        active = not bool(getattr(app, "camera_base_calibration_active", False))
        app.camera_base_calibration_active = active
        set_chessboard_preview_for_camera_base_calibration(active)
        app.camera_calibration_toggle.configure(
            text=f"Calibration Click: {'ON' if active else 'OFF'}",
            fg_color=UI_SUCCESS if active else UI_SURFACE_HIGH,
        )
        point_count = len(getattr(app, "camera_base_points", []))
        pending = getattr(app, "camera_base_pending_pixel", None)
        if active and pending is not None:
            app.camera_base_point_status.configure(
                text=(
                    f"Pending pixel=({pending[0]:.1f},{pending[1]:.1f}); "
                    "jog TCP/pointer to the same physical point, then Add"
                )
            )
        elif active:
            app.camera_base_point_status.configure(
                text=(
                    f"Calibration click ON; {point_count}/9 points saved. "
                    "Click the physical point in the live feed"
                )
            )
        else:
            app.camera_base_point_status.configure(
                text=f"Calibration click OFF; {point_count}/9 points saved"
            )
        app._last_vision_image_update = 0
        research_logger.record("camera_base_click_mode", 1 if active else 0)

    def toggle_camera_base_points_visibility():
        checkbox = getattr(app, "camera_base_points_visible_checkbox", None)
        visible = bool(checkbox.get()) if checkbox is not None else True
        app.camera_base_points_visible = visible

        cfg = load_config()
        cfg.setdefault("vision", {})["camera_to_base_points_visible"] = visible
        save_config(cfg)

        app._last_vision_image_update = 0
        update_camera_base_point_status(
            f"Camera-to-Base points {'shown' if visible else 'hidden'}"
        )
        research_logger.record("camera_base_points_visible", 1 if visible else 0)

    def capture_camera_base_pixel(event):
        if not bool(getattr(app, "camera_base_calibration_active", False)):
            app.camera_base_point_status.configure(
                text="Calibration click is OFF; turn it ON before selecting a point"
            )
            return
        vision_state = read_state("vision", {})
        frame_size = vision_state.get("frame_size", [0, 0]) if isinstance(vision_state, dict) else [0, 0]
        if len(frame_size) < 2 or int(frame_size[0]) <= 0 or int(frame_size[1]) <= 0:
            shared_string.value = b'Error: Camera-to-Base needs an active camera frame'
            return
        display_size = getattr(app, "_vision_display_size", None)
        if not display_size:
            shared_string.value = b'Error: Camera-to-Base live image is not displayed'
            return
        display_w, display_h = [float(value) for value in display_size]
        composed_size = vision_state.get("display_frame_size", frame_size)
        if len(composed_size) < 2 or int(composed_size[0]) <= 0 or int(composed_size[1]) <= 0:
            composed_size = frame_size
        composed_w, composed_h = [float(value) for value in composed_size]
        click_widget = getattr(event, "widget", app.vision_feed_label)
        label_w = float(max(click_widget.winfo_width(), 1))
        label_h = float(max(click_widget.winfo_height(), 1))
        origin_x = (label_w - display_w) / 2.0
        origin_y = (label_h - display_h) / 2.0
        local_x = float(event.x) - origin_x
        local_y = float(event.y) - origin_y
        camera_display_w = display_w * float(frame_size[0]) / composed_w
        camera_display_h = display_h * float(frame_size[1]) / composed_h
        if (
            local_x < 0
            or local_y < 0
            or local_x > camera_display_w
            or local_y > camera_display_h
        ):
            shared_string.value = b'Error: Click inside the camera image, not the mask preview'
            return
        pixel_u = local_x * composed_w / display_w
        pixel_v = local_y * composed_h / display_h
        app.camera_base_pending_pixel = [pixel_u, pixel_v]
        selected_index = selected_camera_base_point_index()
        point_count = len(getattr(app, "camera_base_points", []))
        target_text = (
            f"overwrite P{selected_index + 1}"
            if selected_index is not None
            else "select a saved point to overwrite"
            if point_count >= 9
            else f"add P{point_count + 1}"
        )
        app.camera_base_point_status.configure(
            text=(
                f"Pending pixel=({pixel_u:.1f},{pixel_v:.1f}); "
                f"jog TCP/pointer to the same physical point, then {target_text}"
            )
        )
        app._last_vision_image_update = 0

    def draw_camera_base_calibration_overlay(image, frame_size, display_frame_size):
        points_visible = bool(getattr(app, "camera_base_points_visible", True))
        visible_points = (
            getattr(app, "camera_base_points", [])
            if points_visible
            else []
        )
        pending = (
            getattr(app, "camera_base_pending_pixel", None)
            if bool(getattr(app, "camera_base_calibration_active", False))
            else None
        )
        has_points = bool(visible_points)
        has_pending = pending is not None
        if not has_points and not has_pending:
            return image
        if len(frame_size) < 2 or int(frame_size[0]) <= 0 or int(frame_size[1]) <= 0:
            return image
        if (
            len(display_frame_size) < 2
            or int(display_frame_size[0]) <= 0
            or int(display_frame_size[1]) <= 0
        ):
            display_frame_size = frame_size

        marked = image.copy()
        draw = ImageDraw.Draw(marked)
        scale_x = float(marked.width) / float(display_frame_size[0])
        scale_y = float(marked.height) / float(display_frame_size[1])
        radius = max(7, int(min(marked.size) * 0.014))
        line_width = max(2, radius // 4)

        def draw_marker(pixel, color, label):
            x = float(pixel[0]) * scale_x
            y = float(pixel[1]) * scale_y
            draw.ellipse(
                (x - radius, y - radius, x + radius, y + radius),
                outline=color,
                width=line_width,
            )
            draw.line((x - radius * 1.4, y, x + radius * 1.4, y), fill=color, width=line_width)
            draw.line((x, y - radius * 1.4, x, y + radius * 1.4), fill=color, width=line_width)
            draw.text((x + radius + 4, y - radius), label, fill=color)

        selected_index = getattr(app, "camera_base_selected_index", None)
        for index, point in enumerate(visible_points, start=1):
            pixel = point.get("pixel") if isinstance(point, dict) else None
            if pixel is not None and len(pixel) >= 2:
                color = (255, 128, 0) if selected_index == index - 1 else (0, 255, 255)
                label = f"P{index}*" if selected_index == index - 1 else f"P{index}"
                draw_marker(pixel, color, label)

        if pending is not None and len(pending) >= 2:
            draw_marker(pending, (255, 215, 0), "PENDING")
        return marked

    def render_vision_feed(label, vision_state=None):
        if label is not getattr(app, "vision_feed_label", None):
            return False
        try:
            if not label.winfo_exists():
                return False
        except (tk.TclError, RuntimeError):
            return False

        image = vision_manager.latest_frame_image()
        if image is None:
            return False
        vision_state = vision_state if isinstance(vision_state, dict) else read_state("vision", {})
        frame_size = vision_state.get("frame_size", [0, 0])
        display_frame_size = vision_state.get("display_frame_size", frame_size)
        image = draw_camera_base_calibration_overlay(
            image,
            frame_size,
            display_frame_size,
        )
        ctk_image = customtkinter.CTkImage(
            light_image=image,
            dark_image=image,
            size=image.size,
        )
        label._vision_ctk_image = ctk_image
        app._vision_ctk_image = ctk_image
        widget_scaling = label._get_widget_scaling()
        app._vision_display_size = ctk_image._get_scaled_size(widget_scaling)
        label.configure(image=ctk_image, text="")
        app._last_vision_image_update = time.monotonic()
        return True

    def start_vision_feed_loop(label):
        def update_feed():
            if label is not getattr(app, "vision_feed_label", None):
                return
            try:
                if not label.winfo_exists():
                    return
                if getattr(app, "current_menu", "") == "Vision" or vision_window_is_detached():
                    render_vision_feed(label)
                label.after(100, update_feed)
            except (tk.TclError, RuntimeError):
                return
            except Exception as exc:
                logging.warning("Vision live-feed refresh failed: %s", exc)
                try:
                    label.after(250, update_feed)
                except (tk.TclError, RuntimeError):
                    pass

        label.after_idle(update_feed)

    def refresh_camera_base_points():
        if not hasattr(app, "camera_base_points_text"):
            return
        selected_index = getattr(app, "camera_base_selected_index", None)
        points = getattr(app, "camera_base_points", [])
        if (
            not isinstance(selected_index, int)
            or selected_index < 0
            or selected_index >= len(points)
        ):
            selected_index = None
            app.camera_base_selected_index = None
        try:
            app.camera_base_points_text.configure(state="normal")
        except (tk.TclError, RuntimeError):
            pass
        app.camera_base_points_text.delete("1.0", tk.END)
        try:
            app.camera_base_points_text.tag_config(
                "selected_point",
                background="#1f6aa5",
                foreground="#ffffff",
            )
        except (tk.TclError, RuntimeError):
            pass
        for index, point in enumerate(points, start=1):
            pixel = point["pixel"]
            base = point["base"]
            marker = ">" if selected_index == index - 1 else " "
            app.camera_base_points_text.insert(
                tk.END,
                (
                    f"{marker}{index:02d}: pixel=({pixel[0]:.1f},{pixel[1]:.1f}) "
                    f"Base=({base[0]:.3f},{base[1]:.3f})\n"
                ),
            )
            if selected_index == index - 1:
                try:
                    app.camera_base_points_text.tag_add(
                        "selected_point",
                        f"{index}.0",
                        f"{index}.end",
                    )
                except (tk.TclError, RuntimeError):
                    pass
        try:
            app.camera_base_points_text.configure(state="disabled")
        except (tk.TclError, RuntimeError):
            pass

    def normalize_camera_base_points(points):
        normalized = []
        for point in points if isinstance(points, list) else []:
            if not isinstance(point, dict):
                continue
            pixel = point.get("pixel")
            base = point.get("base")
            if pixel is None and "pixel_point" in point:
                pixel = point.get("pixel_point")
            if base is None and "base_point_mm" in point:
                base = point.get("base_point_mm")
            if (
                isinstance(pixel, (list, tuple))
                and isinstance(base, (list, tuple))
                and len(pixel) >= 2
                and len(base) >= 2
            ):
                try:
                    normalized.append(
                        {
                            "pixel": [float(pixel[0]), float(pixel[1])],
                            "base": [float(base[0]), float(base[1])],
                        }
                    )
                except (TypeError, ValueError):
                    pass
        return normalized[:9]

    def load_camera_base_points_from_config():
        vision_cfg = load_config().get("vision", {})
        points = normalize_camera_base_points(vision_cfg.get("camera_to_base_points", []))
        if points:
            return points

        camera_to_base = vision_cfg.get("camera_to_base", {})
        if not isinstance(camera_to_base, dict):
            return []
        pixel_points = camera_to_base.get("raw_pixel_points") or camera_to_base.get("pixel_points") or []
        base_points = camera_to_base.get("base_points_mm") or []
        rebuilt = []
        for pixel, base in zip(pixel_points, base_points):
            rebuilt.append({"pixel": pixel, "base": base})
        return normalize_camera_base_points(rebuilt)

    def save_camera_base_points_to_config():
        cfg = load_config()
        vision_cfg = cfg.setdefault("vision", {})
        vision_cfg["camera_to_base_points"] = normalize_camera_base_points(
            list(getattr(app, "camera_base_points", []))
        )
        save_config(cfg)

    def selected_camera_base_point_index():
        index = getattr(app, "camera_base_selected_index", None)
        points = getattr(app, "camera_base_points", [])
        if isinstance(index, int) and 0 <= index < len(points):
            return index
        app.camera_base_selected_index = None
        return None

    def camera_base_current_tcp_xy():
        q_current = np.array(
            [PAROL6_ROBOT.STEPS2RADS(Position_in[index], index) for index in range(6)],
            dtype=float,
        )
        transform = PAROL6_ROBOT.robot.fkine(q_current)
        tcp_mm = np.asarray(transform.t, dtype=float) * 1000.0
        return [float(tcp_mm[0]), float(tcp_mm[1])]

    def update_camera_base_point_status(message=None):
        if not hasattr(app, "camera_base_point_status"):
            return
        if message is not None:
            app.camera_base_point_status.configure(text=message)
            return
        selected_index = selected_camera_base_point_index()
        selected_text = f"; selected P{selected_index + 1}" if selected_index is not None else ""
        app.camera_base_point_status.configure(
            text=(
                f"Calibration click {'ON' if app.camera_base_calibration_active else 'OFF'}; "
                f"{len(getattr(app, 'camera_base_points', []))}/9 points saved"
                f"{selected_text}"
            )
        )

    def select_camera_base_point_from_text(event):
        if not hasattr(app, "camera_base_points_text"):
            return
        points = getattr(app, "camera_base_points", [])
        if not points:
            app.camera_base_selected_index = None
            update_camera_base_point_status()
            return
        try:
            text_index = app.camera_base_points_text.index(f"@{event.x},{event.y}")
            line_index = int(str(text_index).split(".")[0]) - 1
        except (ValueError, tk.TclError, RuntimeError):
            return
        if 0 <= line_index < len(points):
            if selected_camera_base_point_index() == line_index:
                app.camera_base_selected_index = None
                update_camera_base_point_status("Point selection cleared")
            else:
                app.camera_base_selected_index = line_index
                update_camera_base_point_status(
                    (
                        f"Selected P{line_index + 1}; click a new pixel if needed, "
                        "then Add / Overwrite TCP"
                    )
                )
        else:
            app.camera_base_selected_index = None
            update_camera_base_point_status()
        refresh_camera_base_points()
        app._last_vision_image_update = 0

    def add_camera_base_point():
        pending = getattr(app, "camera_base_pending_pixel", None)
        selected_index = selected_camera_base_point_index()
        points = getattr(app, "camera_base_points", [])
        if pending is None and selected_index is None:
            shared_string.value = b'Error: Click a camera point before adding Base point'
            return
        if pending is None and selected_index is not None:
            pending = points[selected_index].get("pixel")
        if len(points) >= 9 and selected_index is None:
            shared_string.value = b'Error: 9 points saved; select a point to overwrite or delete one'
            update_camera_base_point_status(
                "9/9 points saved; select a point to overwrite or delete one"
            )
            return

        point = {
            "pixel": [float(pending[0]), float(pending[1])],
            "base": camera_base_current_tcp_xy(),
        }
        if selected_index is not None:
            points[selected_index] = point
            action = f"Overwrote P{selected_index + 1}"
        else:
            points.append(point)
            app.camera_base_selected_index = None
            action = f"Added P{len(points)}"
        app.camera_base_points = points
        app.camera_base_pending_pixel = None
        update_camera_base_point_status(
            (
                f"{action}; {len(app.camera_base_points)}/9 points saved. "
                "Select a point to overwrite or delete"
            )
        )
        save_camera_base_points_to_config()
        refresh_camera_base_points()
        app._last_vision_image_update = 0

    def delete_selected_camera_base_point():
        selected_index = selected_camera_base_point_index()
        if selected_index is None:
            shared_string.value = b'Error: Select a Camera-to-Base point before deleting'
            update_camera_base_point_status("Select a point in the list before deleting")
            return
        points = list(getattr(app, "camera_base_points", []))
        deleted_number = selected_index + 1
        del points[selected_index]
        app.camera_base_points = points
        app.camera_base_pending_pixel = None
        app.camera_base_selected_index = (
            selected_index if selected_index < len(points) else len(points) - 1
        )
        if app.camera_base_selected_index < 0:
            app.camera_base_selected_index = None
        update_camera_base_point_status(
            f"Deleted P{deleted_number}; {len(app.camera_base_points)}/9 points saved"
        )
        save_camera_base_points_to_config()
        refresh_camera_base_points()
        app._last_vision_image_update = 0

    def reset_camera_base_points():
        app.camera_base_points = []
        app.camera_base_pending_pixel = None
        app.camera_base_selected_index = None
        update_camera_base_point_status()
        save_camera_base_points_to_config()
        refresh_camera_base_points()
        app._last_vision_image_update = 0

    def solve_camera_to_base():
        points = list(getattr(app, "camera_base_points", []))
        if len(points) < 9:
            shared_string.value = f"Error: Camera-to-Base needs 9 points; got {len(points)}".encode("utf-8")[:99]
            return
        try:
            result = vision_manager.run_camera_to_base_calibration(
                [point["pixel"] for point in points],
                [point["base"] for point in points],
            )
        except Exception as exc:
            shared_string.value = f"Error: Camera-to-Base failed {exc}".encode("utf-8")[:99]
            research_logger.record("camera_to_base_error", 1, str(exc))
            return
        app.camera_base_point_status.configure(
            text=(
                f"Camera-to-Base valid: RMS={result['rms_error_mm']:.3f} mm, "
                f"max={result['max_error_mm']:.3f} mm"
            )
        )
        save_camera_base_points_to_config()
        shared_string.value = b'Log: Camera-to-Base calibration complete'
        research_logger.record("camera_to_base_calibration", 1, str(result))

    def capture_vision_snapshot():
        try:
            save_vision_settings()
            count = vision_manager.capture_snapshot()
            summary = vision_manager.calibration_summary()
            snapshot_path = str(summary.get("last_snapshot_path", ""))
            valid = bool(summary.get("last_snapshot_valid", False))
            filename = os.path.basename(snapshot_path) if snapshot_path else "-"
            shared_string.value = (
                f"Log: Snapshot {count} {'FOUND' if valid else 'NO BOARD'} {filename}"
            ).encode("utf-8")[:99]
            research_logger.record("vision_calibration_snapshot", count)
        except Exception as exc:
            shared_string.value = f"Error: Snapshot failed {exc}".encode("utf-8")[:99]

    def run_vision_calibration():
        save_vision_settings()
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

    def reset_vision_intrinsic():
        confirmed = messagebox.askyesno(
            "Reset Intrinsic Calibration",
            (
                "Hapus hasil intrinsic, seluruh snapshot calibration, dan "
                "Camera-to-Base calibration?\n\n"
                "Pengaturan Chessboard dan Square mm tetap dipertahankan."
            ),
        )
        if not confirmed:
            return
        try:
            save_vision_settings()
            summary = vision_manager.reset_intrinsic_calibration(delete_snapshots=True)
            app.camera_base_points = []
            app.camera_base_pending_pixel = None
            app.camera_base_selected_index = None
            save_camera_base_points_to_config()
            refresh_camera_base_points()
            app.camera_base_point_status.configure(
                text="Calibration click OFF; 0/9 points saved"
            )
            app.camera_base_calibration_active = False
            app.camera_calibration_toggle.configure(
                text="Calibration Click: OFF",
                fg_color=UI_SURFACE_HIGH,
            )
            app._last_vision_image_update = 0
            deleted = int(summary.get("deleted_snapshots", 0))
            shared_string.value = (
                f"Log: Intrinsic reset; {deleted} snapshots deleted"
            ).encode("utf-8")[:99]
            research_logger.record("vision_intrinsic_reset", deleted)
        except Exception as exc:
            shared_string.value = (
                f"Error: Intrinsic reset failed {exc}"
            ).encode("utf-8")[:99]
            research_logger.record("vision_intrinsic_reset_error", 1, str(exc))

    def run_vision_pick_now():
        save_vision_settings()
        try:
            sequence = build_vision_pick_sequence(shared_string, program_log_queue)
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

    def _format_modbus_timestamp(epoch_value):
        try:
            return datetime.fromtimestamp(float(epoch_value)).strftime("%H:%M:%S.%f")[:-3]
        except Exception:
            return ""

    def _modbus_trigger_packet_meta(modbus_state):
        state = modbus_state if isinstance(modbus_state, dict) else {}
        description = str(state.get("description", ""))
        connected = bool(state.get("connected", False))
        if description.upper() == "MOCK":
            packet_status = "OK"
        elif "timeout" in description.lower():
            packet_status = "Timeout"
        elif not connected:
            packet_status = "Loss"
        else:
            packet_status = "OK"
        try:
            response_time_ms = round(float(state.get("cycle_ms", 0.0)), 3)
        except Exception:
            response_time_ms = 0.0
        return response_time_ms, packet_status

    def _write_modbus_samples_workbook(path, samples, sheet_title, columns, headings, meta):
        ext = os.path.splitext(path)[1].lower()
        iso_columns = {"timestamp"}
        if ext == ".csv":
            import csv as _csv
            with open(path, "w", newline="", encoding="utf-8") as fh:
                writer = _csv.writer(fh)
                writer.writerow([headings.get(c, c) for c in columns])
                for sample in samples:
                    row = []
                    for c in columns:
                        v = sample.get(c, "")
                        if c in iso_columns and v not in ("", None):
                            try:
                                v = datetime.fromtimestamp(float(v)).isoformat(timespec="milliseconds")
                            except Exception:
                                pass
                        row.append(v)
                    writer.writerow(row)
            return path
        from openpyxl import Workbook
        wb = Workbook()
        ws = wb.active
        ws.title = sheet_title[:31] if sheet_title else "Samples"
        ws.append([headings.get(c, c) for c in columns])
        for sample in samples:
            row = []
            for c in columns:
                v = sample.get(c, "")
                if c in iso_columns and v not in ("", None):
                    try:
                        v = datetime.fromtimestamp(float(v)).isoformat(timespec="milliseconds")
                    except Exception:
                        pass
                row.append(v)
            ws.append(row)
        summary_ws = wb.create_sheet("Summary")
        summary_ws.append(["Field", "Value"])
        for key in ("started_at", "finished_at"):
            value = meta.get(key)
            if value:
                try:
                    value = datetime.fromtimestamp(float(value)).isoformat(timespec="milliseconds")
                except Exception:
                    pass
            summary_ws.append([key, value if value is not None else ""])
        for key in ("target", "completed", "stopped_early"):
            summary_ws.append([key, meta.get(key, "")])
        stats = meta.get("stats", {}) or {}
        for key, value in stats.items():
            summary_ws.append([f"stat.{key}", value])
        settings = meta.get("settings", {}) or {}
        for key, value in settings.items():
            summary_ws.append([f"setting.{key}", value])
        wb.save(path)
        return path

    def export_modbus_block_a_xlsx():
        samples = modbus_manager.get_block_a_full_samples()
        if not samples:
            messagebox.showwarning("Export Blok A", "Belum ada data. Jalankan test Blok A terlebih dahulu.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".xlsx",
            initialfile=f"modbus_block_a_{datetime.now():%Y%m%d_%H%M%S}.xlsx",
            filetypes=(("Excel", "*.xlsx"), ("CSV", "*.csv")),
        )
        if not path:
            return
        meta = modbus_manager.get_block_a_full_meta()
        if not meta.get("stats"):
            meta["stats"] = read_state("modbus_block_a", {}).get("stats", {})
        try:
            exported = _write_modbus_samples_workbook(
                path,
                samples,
                "Block A Samples",
                ("n", "timestamp", "response_ms", "status", "detail"),
                {"n": "N", "timestamp": "Timestamp", "response_ms": "Response (ms)", "status": "Status", "detail": "Detail"},
                meta,
            )
        except Exception as exc:
            messagebox.showerror("Export Blok A", f"Gagal export: {exc}")
            return
        research_logger.record("modbus_block_a_export", 1, str(exported))
        messagebox.showinfo("Export Blok A", f"Berhasil disimpan ke:\n{exported}")

    def export_modbus_block_b_xlsx():
        samples = modbus_manager.get_block_b_full_samples()
        if not samples:
            messagebox.showwarning("Export Blok B", "Belum ada data. Jalankan test Blok B terlebih dahulu.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".xlsx",
            initialfile=f"modbus_block_b_{datetime.now():%Y%m%d_%H%M%S}.xlsx",
            filetypes=(("Excel", "*.xlsx"), ("CSV", "*.csv")),
        )
        if not path:
            return
        meta = modbus_manager.get_block_b_full_meta()
        if not meta.get("stats"):
            meta["stats"] = read_state("modbus_block_b", {}).get("stats", {})
        try:
            exported = _write_modbus_samples_workbook(
                path,
                samples,
                "Block B Samples",
                ("index", "timestamp", "status", "duration_s", "note"),
                {"index": "Index", "timestamp": "Timestamp", "status": "Status", "duration_s": "Duration (s)", "note": "Note"},
                meta,
            )
        except Exception as exc:
            messagebox.showerror("Export Blok B", f"Gagal export: {exc}")
            return
        research_logger.record("modbus_block_b_export", 1, str(exported))
        messagebox.showinfo("Export Blok B", f"Berhasil disimpan ke:\n{exported}")

    def export_modbus_cycle_log_xlsx():
        rows = modbus_manager.get_modbus_cycle_log()
        if not rows:
            messagebox.showwarning("Export Cycle Log", "Belum ada data cycle log. Jalankan Blok B terlebih dahulu.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".xlsx",
            initialfile=f"modbus_cycle_log_{datetime.now():%Y%m%d_%H%M%S}.xlsx",
            filetypes=(("Excel", "*.xlsx"), ("CSV", "*.csv")),
        )
        if not path:
            return
        try:
            exported = modbus_manager.export_modbus_cycle_log(path)
        except Exception as exc:
            messagebox.showerror("Export Cycle Log", f"Gagal export: {exc}")
            return
        research_logger.record("modbus_cycle_log_export", len(rows), str(exported))
        messagebox.showinfo("Export Cycle Log", f"Berhasil disimpan ke:\n{exported}")

    def clear_modbus_cycle_log():
        rows = modbus_manager.get_modbus_cycle_log()
        if not rows:
            messagebox.showinfo("Clear Cycle Log", "Cycle log sudah kosong.")
            return
        confirmed = messagebox.askyesno(
            "Clear Cycle Log",
            "Hapus data runtime Modbus Cycle Log?\n\nFile CSV/XLSX yang sudah diexport tidak akan dihapus.",
        )
        if not confirmed:
            research_logger.record("modbus_cycle_log_clear_cancelled", 1)
            return
        if not modbus_manager.clear_modbus_cycle_log():
            messagebox.showwarning("Clear Cycle Log", "Stop Blok B terlebih dahulu sebelum clear cycle log.")
            return
        shared_string.value = b'Log: Modbus cycle log cleared'
        research_logger.record("modbus_cycle_log_ui_clear", 1)

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
        detection_enabled = bool(
            vision_state.get("detection_enabled", vision_manager.detection_enabled())
        ) if isinstance(vision_state, dict) else vision_manager.detection_enabled()
        frame_size = vision_state.get("frame_size", [0, 0]) if isinstance(vision_state, dict) else [0, 0]
        status_text = "Status: " + status
        if len(frame_size) >= 2 and int(frame_size[0]) > 0 and int(frame_size[1]) > 0:
            status_text += f" | Camera actual {int(frame_size[0])}x{int(frame_size[1])}"
        status_text += f" | Detection {'ON' if detection_enabled else 'OFF'}"
        app.vision_status.configure(text=status_text)
        if hasattr(app, "vision_detection_on"):
            app.vision_detection_on.configure(
                state="disabled" if detection_enabled else "normal",
                fg_color=UI_SUCCESS if detection_enabled else UI_SURFACE_HIGH,
            )
        if hasattr(app, "vision_detection_off"):
            app.vision_detection_off.configure(
                state="normal" if detection_enabled else "disabled",
                fg_color=UI_DANGER if not detection_enabled else UI_SURFACE_HIGH,
            )
        model_status = str(vision_state.get("model_status", "NOT LOADED"))
        model_input = vision_state.get("model_input_size", [0, 0]) if isinstance(vision_state, dict) else [0, 0]
        if len(model_input) >= 2 and int(model_input[0]) > 0 and int(model_input[1]) > 0:
            model_status += f" | Input {int(model_input[0])}x{int(model_input[1])} letterbox"
        app.vision_model_status.configure(text="Model: " + model_status)
        color_map = {"SAFE": "#7AC922", "VALID": "#7AC922", "MARGINAL": "#F5A623", "MARGIN": "#F5A623", "UNSAFE": "#E84040", "OUT": "#E84040", "UNKNOWN": "#8A9AB8"}
        info_values = None
        safety = "UNKNOWN"
        if isinstance(latest, dict):
            safety = str(latest.get("workspace_status", "UNKNOWN")).upper()
            info_values = {
                "Status": str(latest.get("workspace_message", latest.get("status", "-"))),
                "BBox": str(latest.get("bbox_px", "-")),
                "Centroid": f"px={latest.get('centroid_px', '-')} | Base={latest.get('centroid_world', '-')}",
                "Dimension": f"W={latest.get('width_mm', 0)} mm | H={latest.get('height_mm', 0)} mm",
                "Pick point": f"Base={latest.get('pick_point_world', '-')}",
                "Orientation": str(latest.get("orientation_deg", "-")),
            }
        elif isinstance(bundle, dict) and bundle.get("selongsong_box"):
            box = [float(value) for value in bundle.get("selongsong_box", [])[:4]]
            if len(box) == 4:
                x1, y1, x2, y2 = box
                centroid = [round((x1 + x2) / 2.0, 1), round((y1 + y2) / 2.0, 1)]
                dimension = [round(max(0.0, x2 - x1), 1), round(max(0.0, y2 - y1), 1)]
            else:
                centroid = "-"
                dimension = [0.0, 0.0]
            safety_text = str(bundle.get("pick_safety", "UNKNOWN")).upper()
            confidence = float(bundle.get("conf_selongsong", 0.0))
            safety = safety_text
            info_values = {
                "Status": f"{safety_text} | confidence={confidence:.3f}",
                "BBox": str(bundle.get("selongsong_box", "-")),
                "Centroid": f"px={centroid} | Base={bundle.get('pick_point_base', '-')}",
                "Dimension": f"W={dimension[0]} px | H={dimension[1]} px",
                "Pick point": (
                    f"px={bundle.get('pick_point_px', '-')} | "
                    f"Base={bundle.get('pick_point_base', '-')}"
                ),
                "Orientation": "N/A (model bbox)",
            }

        if info_values is not None:
            app._vision_object_info_cache = {
                "labels": info_values,
                "safety": safety,
                "updated_at": time.time(),
            }
        elif not detection_enabled or status.strip().lower().startswith("stopped"):
            app._vision_object_info_cache = {}
            info_values = {key: "-" for key in app.vision_info_labels}
            safety = "UNKNOWN"
        else:
            cached = getattr(app, "_vision_object_info_cache", {})
            if isinstance(cached, dict) and isinstance(cached.get("labels"), dict):
                info_values = cached["labels"]
                safety = str(cached.get("safety", "UNKNOWN")).upper()
            else:
                info_values = {key: "-" for key in app.vision_info_labels}

        app.vision_safety_badge.configure(
            text=safety,
            text_color=color_map.get(safety, "#8A9AB8"),
        )
        for key, label in app.vision_info_labels.items():
            label.configure(text=str(info_values.get(key, "-")))
        calibration = vision_state.get("calibration", {}) if isinstance(vision_state, dict) else {}
        if isinstance(calibration, dict):
            camera_to_base = calibration.get("camera_to_base", {})
            camera_to_base = camera_to_base if isinstance(camera_to_base, dict) else {}
            chessboard_found = bool(vision_state.get("chessboard_found", False))
            chessboard_pattern = vision_state.get("chessboard_pattern", [0, 0])
            last_snapshot = os.path.basename(
                str(calibration.get("last_snapshot_path", ""))
            ) or "-"
            app.vision_calibration.configure(
                text=(
                    f"Intrinsic: fx={calibration.get('fx', 0)} fy={calibration.get('fy', 0)} "
                    f"snapshots={calibration.get('snapshots_captured', 0)}\n"
                    f"Chessboard {chessboard_pattern}: "
                    f"{'FOUND' if chessboard_found else 'NOT FOUND'} | "
                    f"last={last_snapshot}\n"
                    f"Camera-to-Base: {'VALID' if camera_to_base.get('valid') else 'NOT CALIBRATED'} "
                    f"RMS={camera_to_base.get('rms_error_mm', '-')}"
                )
            )
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

    def start_vision_state_loop(status_label):
        def update_state_ui():
            if status_label is not getattr(app, "vision_status", None):
                return
            try:
                if not status_label.winfo_exists():
                    return
                _update_vision_ui(read_state("vision", {}))
                status_label.after(100, update_state_ui)
            except (tk.TclError, RuntimeError):
                return
            except Exception as exc:
                logging.warning("Vision state refresh failed: %s", exc)
                try:
                    status_label.after(250, update_state_ui)
                except (tk.TclError, RuntimeError):
                    pass

        status_label.after_idle(update_state_ui)

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
        if hasattr(app, "modbus_block_a_log_tree"):
            samples = modbus_manager.get_block_a_full_samples()
            tree = app.modbus_block_a_log_tree
            rendered = getattr(app, "_modbus_block_a_log_rendered", 0)
            if len(samples) < rendered:
                tree.delete(*tree.get_children())
                rendered = 0
            new_rows = samples[rendered:]
            for sample in new_rows:
                detail = str(sample.get("detail", ""))
                if len(detail) > 200:
                    detail = detail[:197] + "..."
                tree.insert("", "end", values=(
                    sample.get("n", ""),
                    _format_modbus_timestamp(sample.get("timestamp")),
                    sample.get("response_ms", ""),
                    sample.get("status", ""),
                    detail,
                ))
            if new_rows:
                tree.yview_moveto(1.0)
            app._modbus_block_a_log_rendered = len(samples)

    def _handle_modbus_block_b(modbus_state):
        snapshot = modbus_state.get("snapshot", {}) if isinstance(modbus_state, dict) else {}
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
            response_time_ms, packet_status = _modbus_trigger_packet_meta(modbus_state)
            if modbus_manager.block_b_cycle_started(response_time_ms, packet_status):
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
        if hasattr(app, "modbus_block_b_log_tree"):
            samples = modbus_manager.get_block_b_full_samples()
            tree = app.modbus_block_b_log_tree
            rendered = getattr(app, "_modbus_block_b_log_rendered", 0)
            if len(samples) < rendered:
                tree.delete(*tree.get_children())
                rendered = 0
            new_rows = samples[rendered:]
            for sample in new_rows:
                note = str(sample.get("note", ""))
                if len(note) > 200:
                    note = note[:197] + "..."
                tree.insert("", "end", values=(
                    sample.get("index", ""),
                    _format_modbus_timestamp(sample.get("timestamp")),
                    sample.get("status", ""),
                    sample.get("duration_s", ""),
                    note,
                ))
            if new_rows:
                tree.yview_moveto(1.0)
            app._modbus_block_b_log_rendered = len(samples)

    def _update_modbus_ui(modbus_state):
        if not hasattr(app, "modbus_status"):
            return
        if not isinstance(modbus_state, dict):
            modbus_state = {}
        snapshot = modbus_state.get("snapshot", {}) if isinstance(modbus_state, dict) else {}
        app.modbus_status.configure(text="Status: " + str(modbus_state.get("description", "Disconnected")))
        _set_tree_rows(app.modbus_monitor_table, [(key, value) for key, value in snapshot.items()])
        _update_modbus_block_a_ui()
        block_b_running = _handle_modbus_block_b(modbus_state)
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


        
    def _program_file_display_name():
        if Now_open_txt:
            return os.path.basename(Now_open_txt)
        return "No .txt file opened"

    def _update_open_txt_preview():
        label = getattr(app, "program_file_label", None)
        if label is None:
            return

        display_name = _program_file_display_name()
        saved_content = getattr(app, "_program_saved_content", None)
        textbox = getattr(app, "textbox_program", None)
        if textbox is not None and saved_content is not None:
            try:
                if textbox.get("1.0", tk.END) != saved_content:
                    display_name += " *"
            except Exception:
                pass
        _set_text(label, "File: " + display_name)

    def _mark_program_saved():
        app._program_saved_content = app.textbox_program.get("1.0", tk.END)
        _update_open_txt_preview()

    def _write_program_file(file_path):
        with open(file_path, "w") as text_file:
            text_file.write(app.textbox_program.get("1.0", tk.END))
        _mark_program_saved()

    def _confirm_save_current_program():
        global Now_open_txt

        if not Now_open_txt:
            return save_as_txt()

        filename = os.path.basename(Now_open_txt)
        if not messagebox.askyesno("Confirm Save", f"Save changes to {filename}?"):
            return False

        _write_program_file(Now_open_txt)
        return True

    def _confirm_discard_or_save_changes():
        saved_content = getattr(app, "_program_saved_content", None)
        if saved_content is None or app.textbox_program.get("1.0", tk.END) == saved_content:
            return True

        answer = messagebox.askyesnocancel(
            "Unsaved Changes",
            "Save changes before opening another .txt file?",
        )
        if answer is None:
            return False
        if answer:
            return _confirm_save_current_program()
        return True

    def highlight_words_program(event):
        # Re-tagging scans the whole textbox for every command word, so only do
        # it when the content actually changed. The periodic Stuff_To_Update
        # calls this 15x/s; without the cache it rescans the full text each tick
        # (a big slowdown, especially when the window is maximized/fullscreen).
        content = app.textbox_program.get("1.0", tk.END)
        if getattr(app, "_program_highlight_cache", None) == content:
            return
        app._program_highlight_cache = content

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

        _update_open_txt_preview()

    def highlight_words_response(event):
        # Same idea as highlight_words_program: skip the full rescan unless the
        # response log text changed since the last highlight pass.
        content = app.textbox_response.get("1.0", tk.END)
        if getattr(app, "_response_highlight_cache", None) == content:
            return
        app._response_highlight_cache = content

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

        app.program_file_label = customtkinter.CTkLabel(app.program_frame, text="File: No .txt file opened", font=customtkinter.CTkFont(size=14), anchor="e")
        app.program_file_label.grid(row=0, column=1, padx=(10, 20), pady=5, sticky="e")

        app.textbox_program = customtkinter.CTkTextbox(app.program_frame, font=customtkinter.CTkFont(size=PROGRAM_TEXT_FONT_SIZE, family='JetBrains Mono'), fg_color=UI_SURFACE_LOW, text_color=UI_ON_SURFACE, border_color=UI_BORDER, border_width=1, corner_radius=6)
        app.textbox_program.grid(row=1, column=0,columnspan=2, padx=(20, 20), pady=(5, 20), sticky="nsew")

        app.textbox_program.bind("<KeyRelease>", highlight_words_program)
        _mark_program_saved()


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
        def clear_response_log():
            global prev_string_shared
            app.textbox_response.delete("1.0", tk.END)
            try:
                prev_string_shared = (shared_string.value).decode("utf-8", errors="ignore")
            except Exception:
                prev_string_shared = ""

        app.response_frame = customtkinter.CTkFrame(app,height = 100, corner_radius=8, fg_color=UI_SURFACE, border_width=1, border_color=UI_BORDER)
        app.response_frame.grid(row=3, column=1, columnspan=2, padx=(5,0), pady=5, sticky="nsew")
        app.response_frame.grid_columnconfigure(0, weight=1)
        app.response_frame.grid_rowconfigure(1, weight=0)
        app.response_frame.grid_rowconfigure(1, weight=1)


        app.program_label = customtkinter.CTkLabel(app.response_frame, text="Response log:", font=customtkinter.CTkFont(size=16))
        app.program_label.grid(row=0, column=0, padx=(10,10), pady=5, sticky="w")

        app.clear_response_log = customtkinter.CTkButton(app.response_frame,text="Clear log", width=80, font = customtkinter.CTkFont(size=16, family='TkDefaultFont'), command=clear_response_log)
        app.clear_response_log.grid(row=0, column=1, padx=(8, 4),pady = 5,sticky="e")

        app.Show_rec_frame = customtkinter.CTkButton(app.response_frame,text="Show received frame", font = customtkinter.CTkFont(size=16, family='TkDefaultFont'))
        app.Show_rec_frame.grid(row=0, column=2, padx=(4, 20),pady = 5,sticky="w")

        app.textbox_response = customtkinter.CTkTextbox(app.response_frame, font=customtkinter.CTkFont(size=LOG_TEXT_FONT_SIZE, family='JetBrains Mono'), fg_color=UI_SURFACE_LOW, text_color=UI_ON_SURFACE, border_color=UI_BORDER, border_width=1, corner_radius=6)
        app.textbox_response.grid(row=1, column=0,columnspan=3, padx=(20, 20), pady=(5, 20), sticky="nsew")
        app.textbox_response.bind("<KeyRelease>", highlight_words_response)


    #app.textbox_program.bind("<KeyRelease>", highlight_words)

    def commands_frames():
        #commands frame
        app.commands_frame = customtkinter.CTkFrame(app,height = 100,width = right_frames_width, corner_radius=8, fg_color=UI_SURFACE, border_width=1, border_color=UI_BORDER)
        app.commands_frame.grid(row=1, column=3, rowspan = 3, padx=(5,5), pady=5, sticky="news")
        app.commands_frame.grid_columnconfigure(0, weight=1)
        app.commands_frame.grid_columnconfigure(1, weight=0)
        app.commands_frame.grid_rowconfigure(2, weight=1)

        command_help = {
            "Joint_space": ("Joint space", "Pilih command gerak joint. Nilai J1-J6 menggunakan satuan derajat."),
            "Cartesian_space": ("Cartesian space", "Pilih command gerak Cartesian. Posisi menggunakan x, y, z dan orientasi Rx, Ry, Rz."),
            "Conditional_stetements": ("Conditional statements", "Kelompok command kondisi. Belum ada command turunan yang tersedia."),
            "Vision": ("Vision", "Command vision mendeteksi target dan menghitung nilai Z Tool tanpa menggerakkan robot."),
            "Modbus": ("Modbus", "Gunakan ModbusRead untuk menunggu input dan ModbusWrite untuk mengubah output."),
            "Research": ("Research", "Gunakan timestamp dan print untuk pencatatan eksperimen dan pesan log."),
            "Begin": ("Begin()", "Contoh:\nBegin()\n\nRekomendasi:\nWajib menjadi baris pertama program."),
            "End": ("End()", "Contoh:\nEnd()\n\nRekomendasi:\nGunakan pada baris terakhir untuk menjalankan program satu kali."),
            "Loop": ("Loop()", "Contoh:\nLoop()\n\nRekomendasi:\nGunakan sebagai pengganti End() agar program mengulang dari awal."),
            "Delay": ("Delay(seconds)", "Contoh:\nDelay(1.0)\n\nRekomendasi:\nGunakan nilai detik yang lebih besar dari interval kontrol."),
            "MoveJoint": (
                "MoveJoint(J1,J2,J3,J4,J5,J6, options)",
                "Contoh:\n"
                "MoveJoint(0,-90,180,0,10,180,t=4)\n\n"
                "Satuan: J1-J6 dalam derajat.\n\n"
                "Batas nominal (nilai batas tepat ditolak):\n"
                "J1: -123.046875 < J1 < 123.046875\n"
                "J2: -145.0088 < J2 < -3.375\n"
                "J3: 107.866 < J3 < 287.8675\n"
                "J4: -105.46975 < J4 < 105.46975\n"
                "J5: -90 < J5 < 90\n"
                "J6: 0 < J6 < 360\n\n"
                "Margin operasional yang disarankan:\n"
                "J1 -122..122, J2 -144..-4, J3 109..286,\n"
                "J4 -104..104, J5 -89..89, J6 1..359.\n\n"
                "Opsi gerakan:\n"
                "t > 0 detik (disarankan >= 0.1), atau v=0..100 dan a=0..100.\n"
                "Jika t diberikan, v dan a diabaikan. Profil: trap atau poly."
            ),
            "MovePose": (
                "MovePose(x,y,z,Rx,Ry,Rz, options)",
                "Contoh:\nMovePose(250,0,200,180,0,180,t=4)\n\nRekomendasi:\nGunakan pose yang sudah diverifikasi dapat dicapai robot."
            ),
            "MoveCart": (
                "MoveCart(x,y,z,Rx,Ry,Rz, options)",
                "Contoh:\n"
                "MoveCart(250,0,200,180,0,180,t=4,trap)\n\n"
                "Satuan: x,y,z dalam mm; Rx,Ry,Rz dalam derajat.\n"
                "Pose bersifat absolut terhadap base robot.\n\n"
                "Batas Cartesian:\n"
                "- Tidak ada batas x, y, atau z yang berdiri sendiri.\n"
                "- Seluruh lintasan harus memiliki solusi IK dan memenuhi batas J1-J6.\n"
                "- Radius sqrt(x^2+y^2+z^2) normalnya harus di bawah 440 mm.\n"
                "- Saat J5 mendekati +/-90 derajat, batas radius turun hingga sekitar 395 mm.\n"
                "- Gunakan radius <= 400 mm sebagai margin operasional.\n"
                "- Rx,Ry,Rz disarankan dinormalisasi ke -180..180 derajat.\n"
                "- Hindari pose singular, terutama wrist dengan J5 dekat 0 derajat.\n\n"
                "Opsi gerakan:\n"
                "Gunakan t > 0 detik (disarankan >= 0.1). v dan a menerima 0..100,\n"
                "tetapi perhitungan durasi Cartesian saat ini lebih aman menggunakan t.\n"
                "Profil: trap atau poly."
            ),
            "MoveCartRelTRF": (
                "MoveCartRelTRF(dx,dy,dz,dRx,dRy,dRz, options)",
                "Contoh:\n"
                "MoveCartRelTRF(0,0,20,0,0,0,t=1)\n\n"
                "Satuan: dx,dy,dz dalam mm; dRx,dRy,dRz dalam derajat.\n"
                "Gerakan relatif mengikuti sumbu lokal tool, bukan sumbu base.\n\n"
                "Batas dan rekomendasi:\n"
                "- Tidak ada batas angka relatif yang tetap karena bergantung pose awal.\n"
                "- Pose akhir dan seluruh lintasan harus lolos IK serta batas J1-J6.\n"
                "- Radius terhadap base tetap dibatasi sekitar 395..440 mm.\n"
                "- Satu langkah disarankan dx,dy,dz antara -20..20 mm.\n"
                "- Satu langkah rotasi disarankan dRx,dRy,dRz antara -5..5 derajat.\n"
                "- Bagi perpindahan besar menjadi beberapa command kecil.\n"
                "- Hindari pose singular, terutama wrist dengan J5 dekat 0 derajat.\n\n"
                "Opsi gerakan:\n"
                "Gunakan t > 0 detik (disarankan >= 0.1). v dan a menerima 0..100,\n"
                "tetapi perhitungan durasi Cartesian saat ini lebih aman menggunakan t.\n"
                "Profil: trap atau poly."
            ),
            "Output": ("Output(channel,state)", "Contoh:\nOutput(1,HIGH)\n\nRekomendasi:\nChannel yang tersedia: 1 atau 2. State: HIGH atau LOW."),
            "Gripper": ("Gripper(position,speed,current)", "Contoh:\nGripper(255,100,120)\n\nRekomendasi:\nPosition/speed: 0-255. Current: 100-1000."),
            "Gripper_cal": ("Gripper_cal()", "Contoh:\nGripper_cal()\n\nRekomendasi:\nJalankan kalibrasi sebelum command gripper jika gripper belum dikalibrasi."),
            "vision": (
                "vision()",
                "Contoh:\nvision()\nprint(\"$vision.z_plus\")\n\n"
                "Menghasilkan $vision.z_plus dan $vision.z_minus. "
                "Perhitungan berjalan otomatis tanpa menekan tombol GUI. "
                "Command ini tidak menggerakkan robot dan dapat dijalankan offline."
            ),
            "ModbusRead": (
                "ModbusRead(selector,value,timeout)",
                "Contoh:\nModbusRead(trigger_pick,HIGH,timeout=5)\n\nRekomendasi:\nTambahkan timeout agar program tidak menunggu tanpa batas."
            ),
            "ModbusWrite": (
                "ModbusWrite(selector,value)",
                "Contoh:\nModbusWrite(cycle_done,HIGH)\n\nRekomendasi:\nGunakan nama selector yang terdaftar pada konfigurasi Modbus."
            ),
            "timestamp": (
                "timestamp(mode,label)",
                'Contoh:\ntimestamp(record,label="pick_done")\n\nRekomendasi:\nMode yang tersedia: start, stop, atau record.'
            ),
            "print": (
                "print(message)",
                'Contoh:\nprint("Z+ Tool = $vision.z_plus mm")\n\n'
                "Placeholder vision dicetak dengan tiga angka desimal."
            ),
        }
        unsupported_help = {
            "Home", "Input", "Get_data", "Timeouts", "SpeedJoint", "SpeedCart"
        }

        app.select_current_position = customtkinter.CTkRadioButton(master=app.commands_frame, text="Current position/Pose",  value=2,command = Current_position)
        app.select_current_position.grid(row=0, column=0, pady=10, padx=20, sticky="we")

        app.collapse_commands_button = customtkinter.CTkButton(
            app.commands_frame,
            text="▶",
            width=28,
            height=28,
            corner_radius=6,
            fg_color=UI_ACCENT_SOFT,
            hover_color=UI_BORDER,
            text_color=UI_ON_SURFACE,
            command=lambda: toggle_layout_panel("right"),
        )
        app.collapse_commands_button.grid(row=0, column=1, padx=(0, 8), pady=8, sticky="ne")

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
                        rowheight=30,
                        fieldbackground=UI_SURFACE_LOW,
                        bordercolor=UI_BORDER,
                        borderwidth=0,
                        font=(FONT_FAMILY_MAIN, COMMAND_TREE_FONT_SIZE))

        style.map('Treeview',
                  background=[('selected', UI_ACCENT)],
                  foreground=[('selected', '#ffffff')])

        style.configure("Treeview.Heading",
                        background=UI_SURFACE_ALT,
                        foreground=UI_ON_SURFACE,
                        relief="flat",
                        font=(FONT_FAMILY_MAIN, COMMAND_TREE_FONT_SIZE,'bold'),
                        )


        style.map("Treeview.Heading",
                    background=[('active', UI_SURFACE_HIGH)])

        app.add_menu_display211 = customtkinter.CTkFrame(master=app.commands_frame,
                                                    corner_radius=15,
                                                    height=100,
                                                    width=0)
        app.add_menu_display211.grid(row=2, column=0, pady=3, padx=5, sticky="nsew")
        app.add_menu_display211.grid_columnconfigure(0, weight=1)
        app.add_menu_display211.grid_rowconfigure(0, weight=1)

        columns = ( 'item')

        app.table = ttk.Treeview(master=app.add_menu_display211,
                        columns=columns,
                        height=12,
                        selectmode='browse',
                        show='tree')
        
        app.table.column("#0", anchor="w", minwidth=180, width=250, stretch=True)
        app.table.column("#1", minwidth=0, width=0, stretch=False)

        app.table.tag_configure("command_group", font=(FONT_FAMILY_MAIN, COMMAND_TREE_FONT_SIZE, 'bold'), foreground=UI_ACCENT)
        app.table.tag_configure("command_item", font=(FONT_FAMILY_MONO, COMMAND_TREE_FONT_SIZE))
        app.command_scrollbar = ttk.Scrollbar(
            app.add_menu_display211,
            orient="vertical",
            command=app.table.yview,
        )
        app.table.configure(yscrollcommand=app.command_scrollbar.set)
        

        #Commands_list = ["Home","Delay","End","Loop","IO","JointVelSet","JointAccSet","JointMove","PoseMove","JointVelMove",
                            #"CartAccSet","CartVelSet","CartLinVelSet","CartAngVelSet","CartMove","CartVelMoveTRF","CartVelMoveWRF"]
        # Commands
        Joint_space = app.table.insert(parent='', index='end', iid=0, text="Joint Space", values="Joint_space", open=True, tags=("command_group",))
        Cart_space = app.table.insert(parent='', index='end', iid=1, text="Cartesian Space", values="Cartesian_space", open=True, tags=("command_group",))
        Conditional_statements = app.table.insert(parent='', index='end', iid=2, text="Conditional Statements", values="Conditional_stetements", open=True, tags=("command_group",))
        Home = app.table.insert(parent='', index='end', iid=3, text="Home", values="Home", tags=("command_item",))
        Delay = app.table.insert(parent='', index='end', iid=4, text="Delay", values="Delay", tags=("command_item",))
        End = app.table.insert(parent='', index='end', iid=5, text="End", values="End", tags=("command_item",))
        Loop = app.table.insert(parent='', index='end', iid=6, text="Loop", values="Loop", tags=("command_item",))
        Begin_ = app.table.insert(parent='', index='end', iid=7, text="Begin", values="Begin", tags=("command_item",))
        Input_var_ = app.table.insert(parent='', index='end', iid=8, text="Input", values="Input", tags=("command_item",))
        Output_var_ = app.table.insert(parent='', index='end', iid=9, text="Output", values="Output", tags=("command_item",))
        Gripper = app.table.insert(parent='', index='end', iid=10, text="Gripper", values="Gripper", tags=("command_item",))
        Gripper_cal = app.table.insert(parent='', index='end', iid=11, text="Gripper_cal", values="Gripper_cal", tags=("command_item",))
        Get_data = app.table.insert(parent='', index='end', iid=12, text="Get_data", values="Get_data", tags=("command_item",))
        Timeouts = app.table.insert(parent='', index='end', iid=13, text="Timeouts", values="Timeouts", tags=("command_item",))
        Vision_cmds = app.table.insert(parent='', index='end', iid=14, text="Vision", values="Vision", open=True, tags=("command_group",))
        Modbus_cmds = app.table.insert(parent='', index='end', iid=15, text="Modbus", values="Modbus", open=True, tags=("command_group",))
        Research_cmds = app.table.insert(parent='', index='end', iid=16, text="Research", values="Research", open=True, tags=("command_group",))

        # Joint space commands
        v1 = app.table.insert(Joint_space, index='end', iid=100, text="MoveJoint", values="MoveJoint", tags=("command_item",))
        v2 = app.table.insert(Joint_space, index='end', iid=101, text="MovePose", values="MovePose", tags=("command_item",))
        v3 = app.table.insert(Joint_space, index='end', iid=102, text="SpeedJoint", values="SpeedJoint", tags=("command_item",))

    
        # Cart space commands
        v1 = app.table.insert(Cart_space, index='end', iid=110, text="MoveCart", values="MoveCart", tags=("command_item",))
        v2 = app.table.insert(Cart_space, index='end', iid=120, text="MoveCartRelTRF", values="MoveCartRelTRF", tags=("command_item",))
        v3 = app.table.insert(Cart_space, index='end', iid=130, text="SpeedCart", values="SpeedCart", tags=("command_item",))

        app.table.insert(Vision_cmds, index='end', iid=140, text="vision", values="vision", tags=("command_item",))
        app.table.insert(Modbus_cmds, index='end', iid=150, text="ModbusRead", values="ModbusRead", tags=("command_item",))
        app.table.insert(Modbus_cmds, index='end', iid=151, text="ModbusWrite", values="ModbusWrite", tags=("command_item",))
        app.table.insert(Research_cmds, index='end', iid=152, text="timestamp", values="timestamp", tags=("command_item",))
        app.table.insert(Research_cmds, index='end', iid=153, text="print", values="print", tags=("command_item",))

        app.table.grid(row=0, column=0, sticky='nsew', padx=(8, 0), pady=8)
        app.command_scrollbar.grid(row=0, column=1, sticky="ns", padx=(0, 8), pady=8)

        app.command_help_title = customtkinter.CTkLabel(
            app.commands_frame,
            text="Contoh & rekomendasi",
            anchor="w",
            font=customtkinter.CTkFont(size=14, weight="bold"),
        )
        app.command_help_title.grid(row=3, column=0, padx=12, pady=(8, 2), sticky="ew")

        app.command_help_text = customtkinter.CTkTextbox(
            app.commands_frame,
            height=150,
            wrap="word",
            font=customtkinter.CTkFont(size=COMMAND_HELP_FONT_SIZE, family="JetBrains Mono"),
            fg_color=UI_SURFACE_LOW,
            text_color=UI_ON_SURFACE,
            border_color=UI_BORDER,
            border_width=1,
            corner_radius=6,
        )
        app.command_help_text.grid(row=4, column=0, padx=12, pady=(0, 12), sticky="ew")

        def show_command_help(command_name):
            title, help_text = command_help.get(
                command_name,
                (
                    f"{command_name}()",
                    "Contoh belum tersedia.\n\nRekomendasi:\nPeriksa dukungan dan format command sebelum menjalankan program.",
                ),
            )
            if command_name in unsupported_help:
                help_text = (
                    "Contoh terverifikasi belum tersedia.\n\n"
                    "Rekomendasi:\nCommand ini belum memiliki handler eksekusi aktif. "
                    "Jangan gunakan pada program produksi sebelum implementasinya diverifikasi."
                )
            app.command_help_title.configure(text=title)
            app.command_help_text.configure(state="normal")
            app.command_help_text.delete("1.0", tk.END)
            app.command_help_text.insert("1.0", help_text)
            app.command_help_text.configure(state="disabled")

        show_command_help("Begin")

        def select(e):
            selected = app.table.identify_row(e.y)
            if not selected:
                return
            app.table.focus(selected)
            app.table.selection_set(selected)
            logging.debug(selected)
            value = app.table.item(selected,'values')
            if not value:
                return
            logging.debug(value[0])
            show_command_help(value[0])
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
                elif value[0] == "print":
                    app.textbox_program.insert(tk.INSERT, "print(\"tes 1\")" + "\n")
                else:
                    app.textbox_program.insert(tk.INSERT, str(value[0]) + "()" +"\n")
          
            elif Current_Custom_pose_select == "Custom":
                if value[0] == "ModbusRead":
                    app.textbox_program.insert(tk.INSERT, "ModbusRead(trigger_pick, HIGH, timeout=5)" + "\n")
                elif value[0] == "ModbusWrite":
                    app.textbox_program.insert(tk.INSERT, "ModbusWrite(cycle_done, HIGH)" + "\n")
                elif value[0] == "timestamp":
                    app.textbox_program.insert(tk.INSERT, "timestamp(label=\"event\", record)" + "\n")
                elif value[0] == "print":
                    app.textbox_program.insert(tk.INSERT, "print(\"tes 1\")" + "\n")
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
        pattern = re.compile(r'\D*(\d+)\D*')
        match = pattern.match(COMPORT_value)
        if match:
            com_number = int(match.group(1))
            General_data[0] = com_number
        else:
            None

        print(General_data[0])
        
        

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
        expand_button = getattr(app, "expand_commands_button", None)
        if expand_button is None:
            return
        if app.layout_collapsed["right"] and not _active_is_feature():
            expand_button.grid(row=1, column=3, rowspan=3, padx=(0, 5), pady=5, sticky="nse")
            expand_button.tkraise()
        else:
            expand_button.grid_remove()

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
        app.expand_commands_button = customtkinter.CTkButton(
            app,
            text="◀",
            width=28,
            height=72,
            corner_radius=6,
            fg_color=UI_ACCENT_SOFT,
            hover_color=UI_BORDER,
            text_color=UI_ON_SURFACE,
            command=lambda: toggle_layout_panel("right"),
        )

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
        _show_commander_frame(app.cart_frame, "Cart")
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
        if not _confirm_discard_or_save_changes():
            return

        text_file = filedialog.askopenfilename(initialdir = Image_path + "/Programs",title = "open text file", filetypes= (("Text Files","*.txt"),))
        if not text_file:
            return

        logging.debug(text_file)
        Now_open_txt = text_file
        app.textbox_program.delete('1.0', tk.END)
        with open(text_file, 'r') as opened_file:
            temp_var = opened_file.read()
        app.textbox_program.insert(tk.END,temp_var)
        _mark_program_saved()
        app._program_highlight_cache = None
        highlight_words_program(None)

    def execute_program():
        # When program start button is pressed:
        # copy the editor content to execute_script.txt without overwriting
        # the .txt file currently opened in the editor.
        # set Button[7] flag to 1
        logging.debug("Execute program")
        global Now_open_txt
        logging.debug(Now_open_txt)
        runtime_file = Image_path + "/Programs/execute_script.txt"
        with open(runtime_file, 'w') as text_file:
            text_file.write(app.textbox_program.get(1.0,tk.END))
            
        # Set flag to 1. Program will try to run
        update_state("program_control", {"state": "RUNNING", "paused": False, "stop_requested": False, "step_requested": 0, "updated_at": time.time()})
        research_logger.record("program_run", 1, Now_open_txt or "unsaved_editor")
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
        _confirm_save_current_program()
        
    def save_as_txt():
        logging.debug("Save as txt")
        global Now_open_txt

        file_path = filedialog.asksaveasfilename(
            initialdir=Image_path + "/Programs",
            defaultextension=".txt",
            filetypes=(("Text Files", "*.txt"),),
            confirmoverwrite=False,
        )
        if file_path:
            filename = os.path.basename(file_path)
            if os.path.exists(file_path):
                if not messagebox.askyesno("Confirm Overwrite", f"Overwrite {filename}?"):
                    return False
            elif not messagebox.askyesno("Confirm Save", f"Save current program as {filename}?"):
                return False
            Now_open_txt = file_path
            _write_program_file(file_path)
            return True
        return False

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
        if vision_window_is_detached():
            app.vision_detached_window.lift()

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
                            background=UI_SURFACE_LOW,
                            foreground=UI_ON_SURFACE,
                            rowheight=30,
                            fieldbackground=UI_SURFACE_LOW,
                            bordercolor=UI_BORDER,
                            borderwidth=0,
                            font=(FONT_FAMILY_MAIN, COMMAND_TREE_FONT_SIZE))

            style.map('Treeview',
                      background=[('selected', UI_ACCENT)],
                      foreground=[('selected', '#ffffff')])

            style.configure("Treeview.Heading",
                            background=UI_SURFACE_ALT,
                            foreground=UI_ON_SURFACE,
                            relief="flat",
                            font=(FONT_FAMILY_MAIN, COMMAND_TREE_FONT_SIZE,'bold'),
                            )

            style.map("Treeview.Heading",
                        background=[('active', UI_SURFACE_HIGH)])

        if(new_appearance_mode == "Light"):
            style = ttk.Style()
            style.theme_use("default")

            style.configure("Treeview",
                            background="#e0e0e0",
                            foreground="black",
                            rowheight=30,
                            fieldbackground="#e0e0e0",
                            bordercolor="#cccccc",
                            borderwidth=0,
                            font=(FONT_FAMILY_MAIN, COMMAND_TREE_FONT_SIZE))

            style.map('Treeview',
                      background=[('selected', UI_ACCENT)],
                      foreground=[('selected', '#ffffff')])

            style.configure("Treeview.Heading",
                            background="#d0d0d0",
                            foreground="black",
                            relief="flat",
                            font=(FONT_FAMILY_MAIN, COMMAND_TREE_FONT_SIZE,'bold'),
                            )

            style.map("Treeview.Heading",
                        background=[('active', "#c0c0c0")])



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

        # Drain the program log queue first. This is the reliable channel for
        # program output (print/vision/...): every message is delivered exactly
        # once, independent of how fast the program runs or the 66 ms poll rate,
        # so no lines are skipped.
        drained_last = None
        if program_log_queue is not None:
            for _ in range(100):  # bounded so the UI stays responsive
                try:
                    entry = program_log_queue.get_nowait()
                except Exception:
                    break
                if isinstance(entry, dict):
                    message = str(entry.get("message", ""))
                    stamp = entry.get("timestamp")
                else:
                    message = str(entry)
                    stamp = None
                if not message:
                    continue
                try:
                    time_string = datetime.fromtimestamp(stamp).strftime("%H:%M:%S") if stamp else datetime.now().strftime("%H:%M:%S")
                except Exception:
                    time_string = datetime.now().strftime("%H:%M:%S")
                app.textbox_response.insert(tk.INSERT, time_string + "--" + message + "\n")
                drained_last = message
            if drained_last is not None:
                app.textbox_response.see(tk.END)

        shared_string_string = (shared_string.value).decode('UTF-8')
        # Suppress the shared_string echo of a message already shown via the
        # queue, otherwise program logs would appear twice.
        if drained_last is not None and shared_string_string == drained_last:
            prev_string_shared = shared_string_string
        elif shared_string_string != prev_string_shared:
            prev_string_shared = shared_string_string
            time_string = datetime.now().strftime("%H:%M:%S")
            app.textbox_response.insert(tk.INSERT,time_string + "--" + shared_string_string + "\n")
        else:
            prev_string_shared = shared_string_string

        # Keep the response log bounded. It is appended to forever otherwise,
        # which makes the periodic highlight rescan and Tk redraw progressively
        # slower (most noticeable once the window is fullscreen).
        RESPONSE_LOG_MAX_LINES = 400
        line_count = int(app.textbox_response.index("end-1c").split(".")[0])
        if line_count > RESPONSE_LOG_MAX_LINES:
            app.textbox_response.delete("1.0", f"{line_count - RESPONSE_LOG_MAX_LINES}.0")

        #app.textbox_response.insert(tk.INSERT,"tesT\n")

        _set_text(app.Input1, "INPUT 1: " + str(InOut_in[0]).rjust(7, ' '))
        _set_text(app.Input2, "INPUT 2: " + str(InOut_in[1]).rjust(7, ' '))
        _set_text(app.ESTOP_STATUS, "ESTOP: " + str(InOut_in[4]).rjust(7, ' '))
        _set_text(app.OUTPUT_1_LABEL, "OUTPUT 1 is: " + str(InOut_out[2]).rjust(7, ' '))
        _set_text(app.OUTPUT_2_LABEL, "OUTPUT 2 is: " + str(InOut_out[3]).rjust(7, ' '))
        robot_connection_state = 0
        if len(General_data) > 3:
            robot_connection_state = General_data[3]

        # Only touch the connection banner when the state changes (it also swaps
        # text colour, so it cannot use the plain _set_text helper).
        if getattr(app, "_conn_state_cache", None) != robot_connection_state:
            app._conn_state_cache = robot_connection_state
            _conn_font = customtkinter.CTkFont(family='Inter', size=15, weight='bold')
            if robot_connection_state == 0:
                app.estop_status.configure(text="\u25cf NOT CONNECTED", text_color=UI_DANGER, font=_conn_font)
            elif robot_connection_state == 1:
                app.estop_status.configure(text="\u25cf CONNECTING", text_color=UI_WARN, font=_conn_font)
            else:
                app.estop_status.configure(text="\u25cf CONNECTED", text_color=UI_SUCCESS, font=_conn_font)


    
    # Tool positions
    # Use ik to calculate

        # The forward kinematics (fkine) + rpy below is the single most expensive
        # thing in this 15x/s loop. The joint readout only changes while the robot
        # is actually moving, so skip the whole recompute + label redraw when the
        # incoming joint positions are unchanged from the previous tick.
        position_changed = any(Position_in[i] != prev_positions[i] for i in range(6))
        if position_changed:
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

            _set_text(app.x_pos, "X: "+ x_value.rjust(7, ' '))
            _set_text(app.y_pos, "Y: "+ y_value.rjust(7, ' '))
            _set_text(app.z_pos, "Z: "+ z_value.rjust(7, ' '))
            _set_text(app.Rx_pos, "Rx: "+ Rx_pos.rjust(7, ' '))
            _set_text(app.Ry_pos, "Ry: "+ Ry_pos.rjust(7, ' '))
            _set_text(app.Rz_pos, "Rz: "+ Rz_pos.rjust(7, ' '))

            # Joint positions
            Joint1_value = str(round(PAROL6_ROBOT.STEPS2DEG(Position_in[0],0),3))
            Joint2_value = str(round(PAROL6_ROBOT.STEPS2DEG(Position_in[1],1),3))
            Joint3_value = str(round(PAROL6_ROBOT.STEPS2DEG(Position_in[2],2),3))
            Joint4_value = str(round(PAROL6_ROBOT.STEPS2DEG(Position_in[3],3),3))
            Joint5_value = str(round(PAROL6_ROBOT.STEPS2DEG(Position_in[4],4),3))
            Joint6_value = str(round(PAROL6_ROBOT.STEPS2DEG(Position_in[5],5),3))

            _set_text(app.theta1, "θ1: " + Joint1_value.rjust(7, ' '))
            _set_text(app.theta2, "θ2: " + Joint2_value.rjust(7, ' '))
            _set_text(app.theta3, "θ3: " + Joint3_value.rjust(7, ' '))
            _set_text(app.theta4, "θ4: " + Joint4_value.rjust(7, ' '))
            _set_text(app.theta5, "θ5: " + Joint5_value.rjust(7, ' '))
            _set_text(app.theta6, "θ6: " + Joint6_value.rjust(7, ' '))

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
        _set_text(app.Velocity_percent, ""+ str(v1).rjust(4, ' ')+ "%")
        _set_text(app.Accel_percent, "" + str(v2).rjust(4, ' ') + "%")

        # Gripper stuff
        # Sliders
        gpos = app.grip_pos_slider.get()
        #Gripper_data_out[0] = int(gpos)
        _set_text(app.grip_pos_percent, ""+ str(gpos))

        gvel = app.grip_speed_slider.get()
        #Gripper_data_out[1] = int(gvel)
        _set_text(app.grip_speed_percent, ""+ str(gvel))

        gcur = app.grip_current_slider.get()
        #Gripper_data_out[2] = int(gcur)
        _set_text(app.grip_current_percent, "" + str(gcur).rjust(0, ' ') + " mA")

        _set_text(app.grip_feedback_pos, "Gripper position feedback is: " + str(round(Gripper_data_in[1],0)).rjust(7, ' '))
        _set_text(app.grip_feedback_current, "Gripper current feedback is: " + str(round(Gripper_data_in[3],0)).rjust(7, ' '))
        _set_text(app.grip_object_detection, "Gripper object detection is: " + str(round(Gripper_data_in[4],0)).rjust(7, ' '))

        #bitfield_list = [Gripper_activate_deactivate,Gripper_action_status,InOut_in[4],Gripper_rel_dir,0,0,0,0] #InOut_in[4] is estop
        bitfield_list = [Gripper_activate_deactivate,Gripper_action_status,not InOut_in[4],Gripper_rel_dir,0,0,0,0] #InOut_in[4] is estop
        fused = PAROL6_ROBOT.fuse_bitfield_2_bytearray(bitfield_list)
        Gripper_data_out[3] = int(fused.hex(),16)

        Gripper_data_byte = PAROL6_ROBOT.split_2_bitfield(Gripper_data_in[4])
        fused_number = (Gripper_data_byte[2] << 1) | Gripper_data_byte[3]
        if(fused_number == 0):
            _set_text(app.grip_object_detection, "Gripper in motion ")
        elif(fused_number == 1):
            _set_text(app.grip_object_detection, "Object detected when closing ")
        elif(fused_number == 2):
            _set_text(app.grip_object_detection, "Object detected when opening ")
        elif(fused_number == 3):
            _set_text(app.grip_object_detection, "Gripper is at position ")

        _set_text(app.grip_cal_status, "Calibration status is: " + str(Gripper_data_byte[7]).rjust(7, ' '))
        _set_text(app.Error_status_grip, "Error status is: " + str(Gripper_data_byte[6]).rjust(7, ' '))
        _set_text(app.Gripper_ID, "Gripper ID is: " + str(Gripper_data_out[5]))

        highlight_words_response(None)
        highlight_words_program(None)
        _update_modbus_ui(read_state("modbus", {}))
        _update_research_ui(read_state("research", {}))
        # If tab is joint jog
        # Update joint sliders (only when the joints actually moved this tick)
        if position_changed:
            for y in range(0,6):
                app.progress_bar_joints[y].set(np.interp(Position_in[y],[PAROL6_ROBOT.Joint_limits_steps[y][0],PAROL6_ROBOT.Joint_limits_steps[y][1]],[0.0,1.0]))
    
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
    General_data = [4,3000000,0,0]
    # Home,Enable,Disable,Clear error,Real_robot,Sim_robot,Demo app,Program executions,Park
    Buttons = [0,0,0,0,1,1,0,0,0]
    
    shared_string = multiprocessing.Array('c', b' ' * 100)

    GUI(shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons)
