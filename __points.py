# points.py
# Station 3 (Robot_3_2026) survey points, from points_2026.xlsx, sheet "robot_3".
# Format: key: (frame, is_origin, fixture, x, y, z, description)

POINTS = {
    # --- World-frame points ---
    1:  ("world", True,  "cup_dispenser",         -589.3, -221.8, 212.9, "CD_Top_Cover fastener (2010 o'clock)"),
    2:  ("world", False, "cup_dispenser",         -680.4,  -80.6, 213.2, "CD_Top_Cover fastener (1345 o'clock)"),
    3:  ("world", True,  "mazzer",                 504.4, -419.7, 319.5, "Mazzer dose hopper lid apex"),
    4:  ("world", False, "mazzer",                 558.4, -389.7, 319.5, "Mazzer dose hopper lid edge (2100 o'clock)"),
    5:  ("world", False, "mazzer",                 562.7, -524.6, 579.2, "Mazzer bean hopper lid apex"),
    6:  ("world", True,  "mazzer_scale",            441.4, -273.5,  41.7, "MS_Top_Cover fastener (left)"),
    7:  ("world", False, "mazzer_scale",            408.7, -292.4,  41.8, "MS_Top_Cover fastener (right)"),
    8:  ("world", True,  "puq",                     384.8,   84.4, 270.9, "PUQ_Body lid apex"),
    9:  ("world", False, "puq",                     344.7,  124.4, 270.9, "PUQ_Body lid edge (2100 o'clock)"),
    10: ("world", True,  "rancilio",               -411.3, -480.2, 348.0, "TopPanel fastener (left front)"),
    11: ("world", False, "rancilio",               -600.5, -370.8, 348.0, "TopPanel fastener (right front)"),
    12: ("world", True,  "rancilio_scale",         -385.8, -330.2,  41.7, "MS_Top_Cover fastener (left)"),
    13: ("world", False, "rancilio_scale",         -418.2, -311.2,  41.5, "MS_Top_Cover fastener (right)"),
    14: ("world", True,  "wdt",                     590.3,  -98.7,  86.0, "DT_Anvil face (centre)"),
    15: ("world", False, "wdt",                     482.1,  -98.7,  37.3, "DT_Plate centre drill hole"),
    56: ("world", True,  "rancilio_tool_cleaner",  -150.8, -549.1, 179.0, "PC_Base front lip (centre)"),
    57: ("world", False, "rancilio_tool_cleaner",  -150.8, -651.0, 198.0, "PC_Body on button centre (raised)"),
    62: ("world", True,  "customer_zone",          -420.0,  150.0,   0.0, "Table_Cushion"),

    # --- Local-frame points ---
    16: ("local", True,  "cup_dispenser",   0.0,    0.0,    0.0,  "CD_Top_Cover fastener (2010 o'clock)"),
    17: ("local", False, "cup_dispenser",   0.0,  142.0,   90.0,  "CD_Top_Cover fastener (1345 o'clock)"),
    18: ("local", False, "cup_dispenser", -10.2,   72.0,  -71.8,  "CD_Front_Index_Pull (open)"),
    19: ("local", False, "cup_dispenser", -10.2,   71.7,  -36.8,  "CD_Front_Index_Pull (shut)"),
    20: ("local", False, "cup_dispenser",-148.7,   71.1,   31.5,  "Supreme_Cup rim (centre)"),

    21: ("local", True,  "mazzer",  0.0,    0.0,    0.0,  "Mazzer dose hopper lid apex"),
    22: ("local", False, "mazzer",-61.73,   0.0,    0.0,  "Mazzer dose hopper lid edge (2100 o'clock)"),
    23: ("local", False, "mazzer",  0.0,  290.83,   0.0,  "Mazzer bean hopper lid apex"),
    24: ("local", False, "mazzer", 78.3, -134.5,  -80.9,  "Mazzer dose lever face (centre)"),
    25: ("local", False, "mazzer", 93.3, -183.0, -159.5,  "Mazzer off button centre (raised)"),
    26: ("local", False, "mazzer", 89.6, -189.5, -143.0,  "Mazzer on button centre (raised)"),

    27: ("local", True,  "mazzer_scale",  0.0,   0.0,   5.0, "MS_Top_Cover fastener (left)"),
    28: ("local", False, "mazzer_scale",  0.0, -38.0,   0.0, "MS_Top_Cover fastener (right)"),
    29: ("local", False, "mazzer_scale",-12.2, -18.8,  15.0, "MS_Ball_Bearing apex"),
    30: ("local", False, "mazzer_scale", 31.5,  21.53,-15.0, "MS_Lock_Lever (left)"),
    31: ("local", False, "mazzer_scale", 31.5, -59.53,-15.0, "MS_Lock_Lever (right)"),

    32: ("local", True,  "puq", 0.0,   0.0,    0.0,   "PUQ_Body lid apex"),
    33: ("local", False, "puq", 0.0, -56.38,   0.0,   "PUQ_Body lid edge (2100 o'clock)"),
    34: ("local", False, "puq", 9.9,  -0.3, -133.5,   "PUQ_Body upper clamp (centre)"),

    35: ("local", True,  "rancilio",  0.0,    0.0,    0.0,  "TopPanel fastener (left front)"),
    36: ("local", False, "rancilio",  0.0,  220.42,   0.0,  "TopPanel fastener (right front)"),
    37: ("local", False, "rancilio", 46.8,   38.3,  -63.2,  "SwitchRocker (lowered)"),
    38: ("local", False, "rancilio",-14.2,   68.7, -145.4,  "GroupGasket (centre)"),

    39: ("local", True,  "rancilio_scale",  0.0,    0.0,   0.0,  "MS_Top_Cover fastener (left)"),
    40: ("local", False, "rancilio_scale",  0.0,  -38.0,   0.0,  "MS_Top_Cover fastener (right)"),
    41: ("local", False, "rancilio_scale",157.52,  -19.0,  24.24,"RS_Pan (centre)"),
    42: ("local", False, "rancilio_scale", 31.5,   21.53, -15.0, "MS_Lock_Lever (left)"),
    43: ("local", False, "rancilio_scale", 31.5,  -59.53, -15.0, "MS_Lock_Lever (right)"),

    44: ("local", True,  "wdt",  0.0,   0.0,    0.0,  "DT_Anvil face (centre)"),
    45: ("local", False, "wdt",108.0,   0.0,  -42.3,  "DT_Plate centre drill hole"),
    46: ("local", False, "wdt", 27.6,  -0.9,   84.0,  "UK_Rotor fastener (1800 o'clock)"),

    47: ("local", True,  "cup_tool",   0.0,   0.0,    0.0,   "ATI-9120-011T (centre)"),
    48: ("local", False, "cup_tool",-104.5,   0.0,  186.62,  "Cup_Holder top face centre (open)"),
    49: ("local", False, "cup_tool", -53.5,   0.0,  186.62,  "Cup_Holder top face centre (shut)"),

    50: ("local", True,  "mazzer_tool",  0.0,  0.0,    0.0,   "ATI-9120-011T (centre)"),
    51: ("local", False, "mazzer_tool",  0.0,  0.0,  102.82,  "YSR-12 cushioning cylinder tip"),
    52: ("local", False, "mazzer_tool",-50.0,  0.0,   67.06,  "Mazzer_Dose_Bar edge"),

    53: ("local", True,  "rancilio_tool",  0.0,  0.0,    0.0,  "ATI-9120-011T (centre)"),
    54: ("local", False, "rancilio_tool",-32.0,  0.0,   28.07, "Rancilio_Tool_Adapter ball bearing indent"),
    55: ("local", False, "rancilio_tool", 28.7,  0.0,  146.3,  "VST_Basket rim (centre)"),
    63: ("local", False, "rancilio_tool",  1.7,  0.0,    0.0,  "VST_Basket rim (centre) stem angle (degrees)"),
    64: ("local", False, "rancilio_tool", 29.3,  0.0,    0.0,  "VST_Basket depth"),

    58: ("local", True,  "rancilio_tool_cleaner",  0.0,    0.0,   0.0, "PC_Base front lip (centre)"),
    59: ("local", False, "rancilio_tool_cleaner",  0.0, -101.7,  17.3, "PC_Body on button centre (raised)"),
    60: ("local", False, "rancilio_tool_cleaner",-45.0,  -51.2,   8.2, "PC_Silicone_Brush (raised)"),
    61: ("local", False, "rancilio_tool_cleaner", 45.0,  -51.2,   8.2, "PC_Bristle_Brush (raised)"),
}