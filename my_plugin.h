#ifndef _my_plugin_h_
#define _my_plugin_h_

// Used to disable some functionality while developing
#define DEBUG 1

static void plugin_settings_save (void);
static void plugin_settings_restore (void);
static coord_data_t get_tool_location(tool_data_t tool);
static void plugin_settings_load (void);
static void reset (void);
static void tool_select (tool_data_t *tool, bool next);
static status_code_t tool_change (parser_state_t *parser_state);
static void report_options (bool newopt);
static void manualToolUnLoad ();
static void manualToolLoad ();
static bool laserBlocked();
static void debug_output(char* message, coord_data_t *target, plan_line_data_t *pl_data);
static bool is_setting_available (const setting_detail_t *setting);
#endif