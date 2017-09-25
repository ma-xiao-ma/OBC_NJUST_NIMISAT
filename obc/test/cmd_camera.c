/*
 * cmd_camera.c
 *
 *  Created on: 2017年7月12日
 *      Author: MWL
 */
#include "command.h"
#include "camera_805.h"


int test_camera_reset(struct command_context * ctx __attribute__((unused)))
{
    Camera_805_reset();
    return CMD_ERROR_NONE;
}


struct command camera_subcommands[] = {
    {
        .name = "reset",
        .help = "camera software reset",
//        .usage = "<index><id><w_len>",
        .handler = test_camera_reset,
    }
};

struct command __root_command camera_commands_master[] =
{
    {
        .name = "camera",
        .help = "805 camera test",
        .chain = INIT_CHAIN(camera_subcommands),
    },
};

void cmd_camera_setup(void)
{
    command_register(camera_commands_master);
}

