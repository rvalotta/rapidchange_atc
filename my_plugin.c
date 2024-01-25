/*
  tool_change.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Manual tool change with option for automatic touch off

  Part of grblHAL

  Copyright (c) 2020-2023 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <string.h>
#include <stdio.h>

#include "hal.h"
#include "motion_control.h"
#include "protocol.h"
#include "grbl/nvs_buffer.h"
#include "grbl/nuts_bolts.h"

#include "my_plugin.h"
typedef enum {
    Motor_Off = 0,
    Motor_CW = 1,
    Motor_CCW = 2
} atc_motor_state_t;

typedef struct {
    char     alignment;
    char     direction;
    uint8_t  number_of_pockets;
    uint16_t pocket_offset;
    float    pocket_1_x_pos;
    float    pocket_1_y_pos;
    char     origin;
    uint16_t tool_engagement_feed_rate;
    uint16_t tool_pickup_rpm;
    uint16_t tool_dropoff_rpm;
    uint16_t tool_z_engagement;
    uint16_t tool_z_traverse;
    uint16_t tool_z_safe_clearance;
    float    tool_z_retract;
    float    tool_start_height;
    bool     tool_setter;
    bool     tool_recognition;
    bool     dust_cover;
    uint16_t toolsetter_offset;
    uint16_t toolsetter_seek_rate;
    uint16_t toolsetter_retreat;
    uint16_t toolsetter_feed_rate;
    uint16_t toolsetter_max_travel;
    float    toolsetter_x_pos;
    float    toolsetter_y_pos;
    float    toolsetter_z_start_pos;
    float    toolsetter_safe_z;
    uint8_t  toolrecognition_input;
    float    toolrecognition_detect_zone_1;
    float    toolrecognition_detect_zone_2;
    uint8_t  dust_cover_axis;
    uint8_t  dust_cover_open_position;
    uint8_t  dust_cover_closed_position;
    uint8_t  dust_cover_output;
    uint8_t  port;
} plugin_settings_t;

static volatile bool execute_posted = false;
static volatile uint32_t spin_lock = 0;
static nvs_address_t nvs_address;
static uint8_t port, n_ports;
static char max_port[4];
static plugin_settings_t my_settings;
static tool_data_t current_tool, *next_tool = NULL;
static driver_reset_ptr driver_reset = NULL;
static on_report_options_ptr on_report_options;
//static coord_data_t offset;

static const setting_group_detail_t user_groups [] = {
    { Group_Root, Group_UserSettings, "RapidChange ATC"}
};

static const setting_detail_t user_settings[] = {
    { 900, Group_UserSettings, "Alignment", "Axis", Format_RadioButtons, "X,Y", NULL, NULL, Setting_IsExtended, &my_settings.alignment, NULL, NULL },
    { 901, Group_UserSettings, "Direction", NULL, Format_RadioButtons, "Positive,Negative", NULL, NULL, Setting_IsExtended, &my_settings.direction, NULL, NULL },
    { 902, Group_UserSettings, "Number of tool pockets", NULL, Format_Int8, "#00", "0", "120", Setting_IsExtended, &my_settings.number_of_pockets, NULL, NULL },
    { 903, Group_UserSettings, "Pocket Offset", "mm", Format_Int16, "###0", "0", "3000", Setting_IsExtended, &my_settings.pocket_offset, NULL, NULL },
    { 904, Group_UserSettings, "Pocket 1 X Position", "mm", Format_Decimal, "-###0.000", "-9999.999", "9999.999", Setting_IsExtended, &my_settings.pocket_1_x_pos, NULL, NULL },
    { 905, Group_UserSettings, "Pocket 1 Y Position", "mm", Format_Decimal, "-###0.000", "-9999.999", "9999.999", Setting_IsExtended, &my_settings.pocket_1_y_pos, NULL, NULL },
    { 906, Group_UserSettings, "Spindle Start Height", "mm", Format_Decimal, "-##0.000", "-999.999", "999.999", Setting_IsExtended, &my_settings.tool_start_height, NULL, NULL },
    { 907, Group_UserSettings, "Z Retract", "mm", Format_Decimal, "-##0.000", "-127.000", "127.000", Setting_IsExtended, &my_settings.tool_z_retract, NULL, NULL },
    { 908, Group_UserSettings, "Tool Engagement Feed Rate", "mm/min", Format_Int16, "###0", "0", "3000", Setting_IsExtended, &my_settings.tool_engagement_feed_rate, NULL, NULL },
    { 909, Group_UserSettings, "Tool Pickup RPM", "rpm", Format_Int16, "###0", "0", "24000", Setting_IsExtended, &my_settings.tool_pickup_rpm, NULL, NULL },
    { 910, Group_UserSettings, "Tool Dropoff RPM", "rpm", Format_Int16, "###0", "0", "24000", Setting_IsExtended, &my_settings.tool_dropoff_rpm, NULL, NULL },
    { 911, Group_UserSettings, "Tool Z Engage", "mm", Format_Decimal, "-##0.000", "-120", "120", Setting_IsExtended, &my_settings.tool_z_engagement, NULL, NULL },
    { 912, Group_UserSettings, "Tool Z Traverse", "mm", Format_Decimal, "-##0.000", "-120", "120", Setting_IsExtended, &my_settings.tool_z_traverse, NULL, NULL },
    { 913, Group_UserSettings, "Tool Z Safe Clearance", "mm", Format_Decimal, "-##0.000", "-120", "120", Setting_IsExtended, &my_settings.tool_z_safe_clearance, NULL, NULL },
    { 914, Group_UserSettings, "Tool Setter", NULL, Format_RadioButtons, "Disabled, Enabled", NULL, NULL, Setting_IsExtended, &my_settings.tool_setter, NULL, NULL },
    { 915, Group_UserSettings, "Tool Recognition", NULL, Format_RadioButtons, "Disabled, Enabled", NULL, NULL, Setting_IsExtended, &my_settings.tool_recognition, NULL, NULL },
    { 916, Group_UserSettings, "Dust Cover", NULL, Format_RadioButtons, "Disabled, Enabled", NULL, NULL, Setting_IsExtended, &my_settings.dust_cover, NULL, NULL },
    { 917, Group_UserSettings, "Setter Tool Offset", NULL, Format_Int8, "##0", "0", "255", Setting_IsExtended, &my_settings.toolsetter_offset, NULL, NULL },
    { 918, Group_UserSettings, "Setter Seek Rate", NULL, Format_Int8, "###0", "0", "5000", Setting_IsExtended, &my_settings.toolsetter_seek_rate, NULL, NULL },
    { 919, Group_UserSettings, "Setter Retreat", NULL, Format_Int8, "##0", "0", "250", Setting_IsExtended, &my_settings.toolsetter_retreat, NULL, NULL },
    { 920, Group_UserSettings, "Setter Feed Rate", NULL, Format_Int16, "###0", "0", "5000", Setting_IsExtended, &my_settings.toolsetter_feed_rate, NULL, NULL },
    { 921, Group_UserSettings, "Setter Max Travel", NULL, Format_Int8, "##0", "0", "250", Setting_IsExtended, &my_settings.toolsetter_max_travel, NULL, NULL },
    { 922, Group_UserSettings, "Setter X Pos", NULL, Format_Decimal, "####0.000", NULL, NULL, Setting_IsExtended, &my_settings.toolsetter_x_pos, NULL, NULL },
    { 923, Group_UserSettings, "Setter Y Pos", NULL, Format_Decimal, "####0.000", NULL, NULL, Setting_IsExtended, &my_settings.toolsetter_y_pos, NULL, NULL },
    { 924, Group_UserSettings, "Setter Z Start Pos", NULL, Format_Decimal, "####0.000", NULL, NULL, Setting_IsExtended, &my_settings.toolsetter_z_start_pos, NULL, NULL },
    { 925, Group_UserSettings, "Setter Safe Z", NULL, Format_Decimal, "####0.000", NULL, NULL, Setting_IsExtended, &my_settings.toolsetter_safe_z, NULL, NULL },
    { 926, Group_UserSettings, "Tool Recognition Input", NULL, Format_Int8, "##0", "0", "250", Setting_IsExtended, &my_settings.toolrecognition_input, NULL, NULL },
    { 927, Group_UserSettings, "Tool Recognition Detect Zone 1", NULL, Format_Decimal, "####0.000", NULL, NULL, Setting_IsExtended, &my_settings.toolrecognition_detect_zone_1, NULL, NULL },
    { 928, Group_UserSettings, "Tool Recognition Detect Zone 2", NULL, Format_Decimal, "####0.000", NULL, NULL, Setting_IsExtended, &my_settings.toolrecognition_detect_zone_2, NULL, NULL },
    { 929, Group_UserSettings, "Dust Cover Axis", NULL, Format_RadioButtons, "Use Output Pin,A-Axis,B-Axis,C-Axis", NULL, NULL, Setting_IsExtended, &my_settings.dust_cover_axis, NULL, NULL },
    { 930, Group_UserSettings, "Dust Cover Open Position", NULL, Format_Int8, "##0", "0", "250", Setting_IsExtended, &my_settings.dust_cover_open_position, NULL, NULL },
    { 931, Group_UserSettings, "Dust Cover Closed Position", NULL, Format_Int8, "##0", "0", "250", Setting_IsExtended, &my_settings.dust_cover_closed_position, NULL, NULL },
    { 932, Group_UserSettings, "Dust Cover Output", NULL, Format_Int8, "##0", "0", "250", Setting_IsExtended, &my_settings.dust_cover_output, NULL, NULL },
    { 933, Group_UserSettings, "Embroidery trigger port", NULL, Format_Int8, "#0", "0", max_port, Setting_NonCore, &my_settings.port, NULL, is_setting_available, { .reboot_required = On } },

};

static const setting_descr_t user_descriptions[] = {
    { 900, "Value: X Axis or Y Axis\\n\\nThe axis along which the tool pockets of the magazine are aligned in the XY plane." },
    { 901, "Value: Positive or Negative\\n\\nThe direction of travel along the alignment axis from pocket 1 to pocket 2, either positive or negative." },
    { 902, "Value: Count\\n\\nThe total number of pockets in the magazine that may be occupied by a tool." },
    { 903, "Value: Distance (mm)\\n\\nThe distance from one pocket to the next when measuring from center to center." },
    { 904, "Value: X Machine Coordinate (mm)\\n\\nThe x axis position referencing the center of the first tool pocket." },
    { 905, "Value: Y Machine Coordinate (mm)\\n\\nThe y axis position referencing the center of the first tool pocket." },


    { 908, "Value: Feed Rate (mm/min)\\n\\nThe feed rate at which the spindle plunges when engaging the clamping nut." },
    { 909, "Value: Spindle Speed (rpm)\\n\\nThe rpm at which to operate the spindle clockwise when engaging the clamping nut while picking up a tool." },
    { 910, "Value: Spindle Speed (rpm)\\n\\nThe rpm at which to operate the spindle counter-clockwise when engaging the clamping nut while dropping a tool." },
    { 911, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position to which the spindle plunges when engaging the clamping nut." },
    { 912, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position at which the spindle traverses the magazine between dropping off and picking up a tool." },
    { 913, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position for safe clearances of all obstacles." },
    { 914, "Value: Enabled or Disabled\\n\\nAllows for enabling or disabling setting the tool offset during a tool change. This can be useful when configuring your magazine or performing diagnostics to shorten the tool change cycle." },
    { 915, "Value: Enabled or Disabled\\n\\nEnables or disables tool recognition as part of an automatic tool change. If tool recognition is included with your magazine, be sure to properly configure the appropriate settings before enabling." },
    { 916, "Value: Enabled or Disabled\\n\\nEnables or disables the dust cover. If a dust cover is included with your magazine, be sure to properly configure the appropriate settings before enabling." },
    { 917, "Value: Distance (mm)\\n\\nThe distance from the surface of the table bed to the top of the tool setter." },
    { 918, "Value: Feed Rate (mm/min)\\n\\nThe feed rate at which the tool seeks the tool setter on the initial straight probe." },
    { 919, "Value: Distance (mm)\\n\\nThe distance to retreat after contact is made with the tool setter during seek mode." },
    { 920, "Value: Feed Rate (mm/min)\\n\\nThe feed rate at which the tool plunges toward the tool setter on the final straight probe, performed after retreating from the initial straight probe." },
    { 921, "Value: Distance (mm)\\n\\nThe maximum distance of travel that should be attempted when probing from Z Seek Start." },
    { 922, "Value: X Machine Coordinate (mm)\\n\\nThe X position referencing the center of the tool setter." },
    { 923, "Value: Y Machine Coordinate (mm)\\n\\nThe Y position referencing the center of the tool setter." },
    { 924, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position at which to begin the initial straight probe." },
    { 925, "Value: Z Machine Coordinate (mm)\\n\\nThe minimum Z position at which it is safe to move above the tool setter with a tool." },
    { 926, "Value: Input Number\\n\\nThe input pin designation for reading the tool recognition sensor state." },
    { 927, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position for recognizing the presence of a clamping nut attached to the spindle." },
    { 928, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position for recognizing the complete threading of a clamping nut after picking up a tool." },
    { 929, "Value: A Axis, B Axis, or C Axis\\n\\nThe axis assigned for dust cover control. This is required to control the dust cover with an axis." },
    { 930, "Value: A, B, or C Machine Coordinate (mm)\\n\\nThe position along the assigned axis at which the dust cover is fully open." },
    { 931, "Value: A, B, or C Machine Coordinate (mm)\\n\\nThe position along the assigned axis at which the dust cover is fully closed." },
    { 932, "Value: Output Number\\n\\nThe output pin designation for dust cover control. This is required to control the dust cover with a third-party microcontroller." },
    { 933, "Testing" }
};

static setting_details_t setting_details = {
    .groups = user_groups,
    .n_groups = sizeof(user_groups) / sizeof(setting_group_detail_t),
    .settings = user_settings,
    .n_settings = sizeof(user_settings) / sizeof(setting_detail_t),
    .save = plugin_settings_save,
    .load = plugin_settings_load,
    .restore = plugin_settings_restore,
    .descriptions = user_descriptions,
    .n_descriptions = sizeof(user_descriptions) / sizeof(setting_descr_t)
};

// Write settings to non volatile storage (NVS).
static void plugin_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&my_settings, sizeof(plugin_settings_t), true);
}

static bool is_setting_available (const setting_detail_t *setting)
{
    bool ok = false;

    switch(setting->id) {

        case Setting_UserDefined_2:
            ok = ioport_can_claim_explicit();
            break;
        default:
            break;
    }

    return ok;
}

// Restore default settings and write to non volatile storage (NVS).
static void plugin_settings_restore (void)
{
    my_settings.alignment = 0;  // 0 = X, 1 = Y
    my_settings.direction = 0;  // 0 = +, 1 = -
    my_settings.number_of_pockets = 0;
    my_settings.pocket_offset = 0;
    my_settings.pocket_1_x_pos = 0.00f;
    my_settings.pocket_1_y_pos = 0.00f;
    my_settings.tool_engagement_feed_rate = 0;
    my_settings.tool_pickup_rpm = 0;
    my_settings.tool_dropoff_rpm = 0;
    my_settings.tool_z_retract = 0;
    my_settings.tool_start_height = 0;
    my_settings.tool_z_engagement = 0;
    my_settings.tool_z_traverse = 0;
    my_settings.tool_z_safe_clearance = 0;
    my_settings.tool_setter = false;
    my_settings.tool_recognition = false;
    my_settings.dust_cover = false;
    my_settings.toolsetter_offset = 0;
    my_settings.toolsetter_seek_rate = 0;
    my_settings.toolsetter_retreat = 0;
    my_settings.toolsetter_feed_rate = 0;
    my_settings.toolsetter_max_travel = 0;
    my_settings.toolsetter_x_pos = 0;
    my_settings.toolsetter_y_pos = 0;
    my_settings.toolsetter_z_start_pos = 0;
    my_settings.toolsetter_safe_z = 0;
    my_settings.toolrecognition_input = 0;
    my_settings.toolrecognition_detect_zone_1 = 0;
    my_settings.toolrecognition_detect_zone_2 = 0;
    my_settings.dust_cover_axis = 0;
    my_settings.dust_cover_open_position = 0;
    my_settings.dust_cover_closed_position = 0;
    my_settings.dust_cover_output = 0;
    my_settings.port = 0;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&my_settings, sizeof(plugin_settings_t), true);
}

// Load settings from volatile storage (NVS)
static void plugin_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&my_settings, nvs_address, sizeof(plugin_settings_t), true) != NVS_TransferResult_OK)
        plugin_settings_restore();
}

// Return X,Y based on tool number
static coord_data_t get_tool_location(tool_data_t tool) {
    coord_data_t target = {0};

    memset(&target, 0, sizeof(coord_data_t)); // Zero plan_data struct

    if(my_settings.alignment == 0) { // X Axis
        if(my_settings.direction == 0) { // Positive
            target.x = my_settings.pocket_1_x_pos + (float) ((tool.tool_id - 1) * my_settings.pocket_offset );
        } else {
            target.x = my_settings.pocket_1_x_pos - (float) ((tool.tool_id - 1) * my_settings.pocket_offset );
        }
        target.y = my_settings.pocket_1_y_pos;
    } else {
        if(my_settings.direction == 0) { // Positive
            target.y = my_settings.pocket_1_y_pos + (float) ((tool.tool_id - 1) * my_settings.pocket_offset );
        } else {
            target.y = my_settings.pocket_1_y_pos - (float) ((tool.tool_id - 1) * my_settings.pocket_offset );
        }
        target.x = my_settings.pocket_1_x_pos;
    }

    return target;
}

//     protocol_buffer_synchronize();


// Reset claimed HAL entry points and restore previous tool if needed on soft restart.
// Called from EXEC_RESET and EXEC_STOP handlers (via HAL).
static void reset (void)
{
    if(next_tool) { //TODO: move to gc_xxx() function?
        // Restore previous tool if reset is during change

        if(current_tool.tool_id != next_tool->tool_id) {
            memcpy(next_tool, &current_tool, sizeof(tool_data_t));
            system_add_rt_report(Report_Tool);
        }

        gc_state.tool_pending = gc_state.tool->tool_id;
        next_tool = NULL;
    }

    driver_reset();
}

// Set next and/or current tool. Called by gcode.c on on a Tn or M61 command (via HAL).
static void tool_select (tool_data_t *tool, bool next)
{
    next_tool = tool;
    if(!next)
        memcpy(&current_tool, tool, sizeof(tool_data_t));
}

static status_code_t spindle(bool load) {

    debug_output(load ? "Loading" : "Unloading", NULL, NULL);
    coord_data_t target = {0}, current_pos;
    plan_line_data_t plan_data;

    if(current_tool.tool_id == 0 && !load) {
        debug_output("No tool to unload", NULL, NULL);
        return Status_OK;
    }

    if(next_tool->tool_id > my_settings.number_of_pockets) {
        debug_output("Tool number is larger than pocket. Manual Tool Change", NULL, NULL);
        if(load) {
            manualToolLoad();
        } else {
            manualToolUnLoad();
        }
        return Status_OK;
    }

    memset(&target, 0, sizeof(coord_data_t)); // Zero plan_data struct
    plan_data_init(&plan_data);

    // Lets stop the spindle and set the feed rate for all moves.
    plan_data.spindle.hal->set_state(plan_data.spindle.hal, (spindle_state_t){0}, 0.0f);
    plan_data.feed_rate = my_settings.tool_engagement_feed_rate;
    plan_data.condition.rapid_motion = Off;


    system_convert_array_steps_to_mpos(current_pos.values, sys.position);
    debug_output("Getting Current POS", &current_pos, &plan_data);

    // Raise Z to safe clearance 
    target = current_pos;
    target.z = my_settings.tool_z_safe_clearance;
    debug_output("Raising Z to Clearance Height", NULL, &plan_data);
    mc_line(target.values, &plan_data);

    // Get X,Y for current tool and move to that position
    target = get_tool_location((load?*next_tool:current_tool));
    target.z = my_settings.tool_z_safe_clearance;
    debug_output("Determine tool position and go there", &target, &plan_data);
    mc_line(target.values, &plan_data);

    target.z = my_settings.tool_start_height;
    debug_output("Going to Spindle Start Height", &target, &plan_data);
    mc_line(target.values, &plan_data);

    // Turn on the spindle CCW
    if(load) {
        plan_data.spindle.hal->set_state(plan_data.spindle.hal, (spindle_state_t){ .on = On }, my_settings.tool_pickup_rpm);
    } else {
        plan_data.spindle.hal->set_state(plan_data.spindle.hal, (spindle_state_t){ .on = On, .ccw = On }, my_settings.tool_dropoff_rpm);
    }

    // move to engagement height
    target.z = my_settings.tool_z_engagement;
    debug_output("Turning on spindle and moving to engagement height", &target, &plan_data);
    mc_line(target.values, &plan_data);

    // Are we doing tool recognition
    if(my_settings.tool_recognition) {
        debug_output("Tool Recognition Enabled", NULL, NULL);
        // Move spindle to zone 2
        target.z = my_settings.toolrecognition_detect_zone_2;
        debug_output("Moving to zone 2", &target, &plan_data);
        mc_line(target.values, &plan_data);
        // Wait for spindle to be in the correct location
        protocol_buffer_synchronize();
        // IF the nut isn't all the way on lets try again
        if(laserBlocked()) {
            target.z = my_settings.tool_z_engagement;
            debug_output("Detection Failed Trying again", NULL, NULL);

            mc_line(target.values, &plan_data);
            target.z = my_settings.toolrecognition_detect_zone_1;
            mc_line(target.values, &plan_data);
            protocol_buffer_synchronize();
        }
        
        if(laserBlocked()) {
            // TODO: Need to error out.
            return Status_GcodeInvalidTarget;
        }

        // Bring Spindle up and Turn off spindle
        
        target.z = my_settings.tool_z_safe_clearance;
        plan_data.spindle.hal->set_state(plan_data.spindle.hal, (spindle_state_t)(spindle_state_t){ .on = Off }, 0.0f);
        debug_output("Stopping spindle and raising to clearance height", &target, &plan_data);
        mc_line(target.values, &plan_data); 

    }
    debug_output("Updating current tool", NULL, NULL);
    
    if(load) {
        memset(&current_tool, 0, sizeof(tool_data_t));
    } else {
        memcpy(&current_tool, next_tool, sizeof(tool_data_t));
    }

    protocol_buffer_synchronize();

    return Status_OK;

}

static void manualToolLoad() {

}

static void manualToolUnLoad() {

}

static void measureTool() {
    coord_data_t current_pos;

    system_convert_array_steps_to_mpos(current_pos.values, sys.position);

}

static bool laserBlocked() {

    return false;
}

// Start a tool change sequence. Called by gcode.c on a M6 command (via HAL).
static status_code_t tool_change (parser_state_t *parser_state)
{
    if(next_tool == NULL)
        return Status_GCodeToolError;

    if(current_tool.tool_id == next_tool->tool_id)
        return Status_OK;

#ifndef DEBUG
    uint8_t homed_req =  (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT);

    if((sys.homed.mask & homed_req) != homed_req)
        return Status_HomingRequired;
#endif

    coord_data_t previous;

    // Save current position
    system_convert_array_steps_to_mpos(previous.values, sys.position);

    debug_output("Turning off Coolant", NULL, NULL);

    // Stop spindle and coolant
    hal.coolant.set_state((coolant_state_t){0});
    
    debug_output("Check if we need to unload tool", NULL, NULL);
    spindle(false);
    debug_output("Check if we need to load a tool", NULL, NULL);
    spindle(true);
    debug_output("Check if we need to measure a tool", NULL, NULL);
    measureTool();
    

    return Status_OK;
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
        hal.stream.write("[PLUGIN: RapidChange ATC v0.01]" ASCII_EOL);
    }        
}

static void warning_mem (uint_fast16_t state)
{
    report_message("Embroidery plugin failed to initialize, no NVS storage for settings!", Message_Warning);
}

// Claim HAL tool change entry points and clear current tool offsets.
void my_plugin_init (void)
{
    hal.driver_cap.atc = On;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;

    if(sys.tlo_reference_set.mask != 0) {
        sys.tlo_reference_set.mask = 0;
        system_add_rt_report(Report_TLOReference);
    }

    gc_set_tool_offset(ToolLengthOffset_Cancel, 0, 0.0f);

    hal.tool.select = tool_select;
    hal.tool.change = tool_change;

    if((nvs_address = nvs_alloc(sizeof(plugin_settings_t)))) {
         settings_register(&setting_details);
    } else {
        protocol_enqueue_rt_command(warning_mem);
    }
 
    if(driver_reset == NULL) {
        driver_reset = hal.driver_reset;
        hal.driver_reset = reset;
    }
}

void debug_output(char* message, coord_data_t *target, plan_line_data_t *pl_data) {

#ifdef DEBUG
    hal.stream.write("[R-ATC]: ");
    hal.stream.write(message);
    hal.stream.write(ASCII_EOL);

    if(target != NULL) {
        hal.stream.write(ASCII_EOL);
        hal.stream.write("Target:" ASCII_EOL);
        hal.stream.write("X: ");
        hal.stream.write( ftoa(target->x, 3) );
        hal.stream.write(ASCII_EOL);
        hal.stream.write("y: ");
        hal.stream.write( ftoa(target->y, 3) );
        hal.stream.write(ASCII_EOL);
        hal.stream.write("z: ");
        hal.stream.write( ftoa(target->z, 3) );
        hal.stream.write(ASCII_EOL);
    }

    if(pl_data != NULL) {
        hal.stream.write(ASCII_EOL "Plan:" ASCII_EOL);
        hal.stream.write("Feed Rate:");
        hal.stream.write(ftoa(pl_data->feed_rate,3));
        hal.stream.write(ASCII_EOL);
        hal.stream.write("Spindle RPM:");
        hal.stream.write(ftoa(pl_data->spindle.rpm,3));
        hal.stream.write(ASCII_EOL);    
        hal.stream.write("Spindle State:");
        char buffer[8U] = ""; /*the output buffer*/ 
        
        sprintf (buffer, "%d", pl_data->spindle.state.value);    
        hal.stream.write(buffer);
        hal.stream.write(ASCII_EOL);

        hal.stream.write(ASCII_EOL);
    }
#endif
}