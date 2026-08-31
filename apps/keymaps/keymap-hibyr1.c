/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) Roman Artiukhin 2025
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/

#include "config.h"
#include "action.h"
#include "button.h"
#include "settings.h"

//#define BUTTON_QS_COMBO (BUTTON_DOWN | BUTTON_LEFT)

/* {Action Code,    Button code,    Prereq button code } */

/* 
 * The format of the list is as follows
 * { Action Code,   Button code,    Prereq button code } 
 * if there's no need to check the previous button's value, use BUTTON_NONE
 * Insert LAST_ITEM_IN_LIST at the end of each mapping 
 */
static const struct button_mapping button_context_standard[]  = {
    { ACTION_STD_PREV,        BUTTON_UP,                BUTTON_NONE },
    { ACTION_STD_PREVREPEAT,  BUTTON_UP|BUTTON_REPEAT,  BUTTON_NONE },

    { ACTION_STD_NEXT,        BUTTON_PREV,                BUTTON_NONE },
    { ACTION_STD_NEXTREPEAT,  BUTTON_PREV|BUTTON_REPEAT,  BUTTON_NONE },

    { ACTION_STD_NEXT,        BUTTON_DOWN,                 BUTTON_NONE },
    { ACTION_STD_NEXTREPEAT,  BUTTON_DOWN|BUTTON_REPEAT,   BUTTON_NONE },

    { ACTION_STD_PREV,        BUTTON_NEXT,                 BUTTON_NONE },
    { ACTION_STD_PREVREPEAT,  BUTTON_NEXT|BUTTON_REPEAT,   BUTTON_NONE },

    { ACTION_STD_MENU,        BUTTON_POWER|BUTTON_REL,            BUTTON_POWER },

    { ACTION_STD_OK,          BUTTON_RIGHT|BUTTON_REL,                      BUTTON_RIGHT },

    { ACTION_STD_CANCEL,      BUTTON_LEFT|BUTTON_REL,      BUTTON_LEFT },
    { ACTION_STD_CANCEL,        BUTTON_POWER|BUTTON_REL,               BUTTON_POWER },
    { ACTION_STD_CANCEL,          BUTTON_PLAY|BUTTON_REL,                      BUTTON_PLAY },

    { ACTION_STD_CONTEXT,     BUTTON_RIGHT|BUTTON_REPEAT,       BUTTON_RIGHT },
    { ACTION_STD_CONTEXT,     BUTTON_PLAY|BUTTON_REPEAT,       BUTTON_PLAY },

    { ACTION_STD_KEYLOCK,       BUTTON_POWER|BUTTON_UP,      BUTTON_NONE },

   // { ACTION_WPS_QUICKSCREEN,       BUTTON_QS_COMBO,      BUTTON_NONE },
    { ACTION_STD_QUICKSCREEN,       BUTTON_POWER|BUTTON_DOWN,      BUTTON_NONE },
    //{ ACTION_WPS_MENU,       BUTTON_POWER|BUTTON_LEFT,      BUTTON_POWER },
    //{ ACTION_WPS_PITCHSCREEN,       BUTTON_POWER|BUTTON_RIGHT,      BUTTON_POWER },

    LAST_ITEM_IN_LIST
}; /* button_context_standard */

static const struct button_mapping button_context_wps[]  = {
    { ACTION_WPS_BROWSE,     BUTTON_UP|BUTTON_REL,        BUTTON_UP },
    //{ ACTION_WPS_STOP,     BUTTON_POWER|BUTTON_REPEAT,     BUTTON_POWER },

    { ACTION_WPS_SKIPPREV, BUTTON_LEFT|BUTTON_REL,    BUTTON_LEFT },
    { ACTION_WPS_SEEKBACK, BUTTON_LEFT|BUTTON_REPEAT, BUTTON_NONE },
    { ACTION_WPS_STOPSEEK, BUTTON_LEFT|BUTTON_REL,    BUTTON_LEFT|BUTTON_REPEAT },

    { ACTION_WPS_SKIPPREV, BUTTON_PREV|BUTTON_REL,    BUTTON_PREV },
    { ACTION_WPS_SEEKBACK, BUTTON_PREV|BUTTON_REPEAT, BUTTON_NONE },
    { ACTION_WPS_STOPSEEK, BUTTON_PREV|BUTTON_REL,    BUTTON_PREV|BUTTON_REPEAT },

    { ACTION_WPS_SKIPNEXT, BUTTON_RIGHT|BUTTON_REL,    BUTTON_RIGHT },
    { ACTION_WPS_SEEKFWD,  BUTTON_RIGHT|BUTTON_REPEAT, BUTTON_NONE },
    { ACTION_WPS_STOPSEEK, BUTTON_RIGHT|BUTTON_REL,    BUTTON_RIGHT|BUTTON_REPEAT },
    
    { ACTION_WPS_SKIPNEXT, BUTTON_NEXT|BUTTON_REL,    BUTTON_NEXT },
    { ACTION_WPS_SEEKFWD,  BUTTON_NEXT|BUTTON_REPEAT, BUTTON_NONE },
    { ACTION_WPS_STOPSEEK, BUTTON_NEXT|BUTTON_REL,    BUTTON_NEXT|BUTTON_REPEAT },

    // { ACTION_WPS_ABSETB_NEXTDIR,    BUTTON_POWER|BUTTON_RIGHT,   BUTTON_POWER },
    // { ACTION_WPS_ABSETA_PREVDIR,    BUTTON_POWER|BUTTON_LEFT,    BUTTON_POWER },
    // { ACTION_WPS_ABRESET,           BUTTON_POWER|BUTTON_UP,      BUTTON_POWER },

    { ACTION_WPS_VOLUP,     BUTTON_UP|BUTTON_REPEAT,     BUTTON_NONE },
    { ACTION_WPS_VOLDOWN,   BUTTON_DOWN|BUTTON_REPEAT,    BUTTON_NONE },

    { ACTION_WPS_PLAY,        BUTTON_POWER|BUTTON_REL,     BUTTON_POWER },
    { ACTION_WPS_PLAY,        BUTTON_PLAY|BUTTON_REL,     BUTTON_PLAY },
    { ACTION_WPS_STOP,   BUTTON_PLAY|BUTTON_REPEAT,    BUTTON_PLAY },

    { ACTION_STD_KEYLOCK,       BUTTON_POWER|BUTTON_UP,      BUTTON_NONE },

//    { ACTION_WPS_QUICKSCREEN,       BUTTON_QS_COMBO,      BUTTON_NONE },
    { ACTION_WPS_QUICKSCREEN,       BUTTON_POWER|BUTTON_DOWN,      BUTTON_NONE },
    { ACTION_WPS_MENU,       BUTTON_POWER|BUTTON_LEFT,      BUTTON_NONE },
    { ACTION_WPS_PITCHSCREEN,       BUTTON_POWER|BUTTON_RIGHT,      BUTTON_NONE },

   { ACTION_WPS_CONTEXT,   BUTTON_POWER|BUTTON_REPEAT,  BUTTON_POWER },
    //TODO: { ACTION_WPS_ID3SCREEN,     BUTTON_SELECT|BUTTON_DOWN,      BUTTON_SELECT },

    { ACTION_WPS_HOTKEY, BUTTON_DOWN|BUTTON_REL,      BUTTON_DOWN },

    LAST_ITEM_IN_LIST
};
/* button_context_wps */
static const struct button_mapping button_context_wps_locked[] = {
    { ACTION_WPS_VOLUP,     BUTTON_UP|BUTTON_REL,     BUTTON_UP },
    { ACTION_WPS_VOLDOWN,   BUTTON_DOWN|BUTTON_REL,    BUTTON_DOWN },
    { ACTION_WPS_STOP,   BUTTON_POWER|BUTTON_REPEAT,    BUTTON_POWER },

    LAST_ITEM_IN_LIST__NEXTLIST(CONTEXT_WPS)
}; /* button_context_wps_locked */

static const struct button_mapping button_context_settings[] = {
        { ACTION_SETTINGS_INC,      BUTTON_UP,                  BUTTON_NONE },
        { ACTION_SETTINGS_INCREPEAT,BUTTON_UP|BUTTON_REPEAT,    BUTTON_NONE },
        { ACTION_SETTINGS_DEC,      BUTTON_DOWN,                BUTTON_NONE },
        { ACTION_SETTINGS_DECREPEAT,BUTTON_DOWN|BUTTON_REPEAT,  BUTTON_NONE },

        { ACTION_STD_PREV,          BUTTON_LEFT | BUTTON_REL,                BUTTON_LEFT },
        { ACTION_STD_PREVREPEAT,    BUTTON_LEFT|BUTTON_REPEAT,  BUTTON_NONE },
        { ACTION_STD_NEXT,          BUTTON_RIGHT | BUTTON_REL,               BUTTON_RIGHT },
        { ACTION_STD_NEXTREPEAT,    BUTTON_RIGHT|BUTTON_REPEAT, BUTTON_NONE },
        { ACTION_STD_OK,    BUTTON_POWER|BUTTON_RIGHT, BUTTON_NONE },
        { ACTION_STD_CANCEL,    BUTTON_POWER|BUTTON_LEFT, BUTTON_NONE },

        LAST_ITEM_IN_LIST__NEXTLIST(CONTEXT_STD),
}; /* button_context_settings */

static const struct button_mapping button_goto_std_context[]  = {
    LAST_ITEM_IN_LIST__NEXTLIST(CONTEXT_STD)
}; 

static const struct button_mapping button_goto_tree_context[]  = {
    LAST_ITEM_IN_LIST__NEXTLIST(CONTEXT_TREE)
};
static const struct button_mapping button_goto_settings_context[]  = {
    LAST_ITEM_IN_LIST__NEXTLIST(CONTEXT_SETTINGS)
}; 

static const struct button_mapping button_goto_settings_right_is_inc_context[]  = {
    LAST_ITEM_IN_LIST__NEXTLIST(CONTEXT_SETTINGS|CONTEXT_CUSTOM)
}; 

static const struct button_mapping button_context_list[]  = {
//    {ACTION_LISTTREE_PGUP,       BUTTON_UP|BUTTON_REL,                  BUTTON_REC|BUTTON_UP},
//    {ACTION_LISTTREE_PGDOWN,       BUTTON_DOWN|BUTTON_REL,                  BUTTON_REC|BUTTON_DOWN},

    LAST_ITEM_IN_LIST__NEXTLIST(CONTEXT_STD)
}; /* button_context_list */

static const struct button_mapping button_context_tree[]  = {
    { ACTION_TREE_STOP,    BUTTON_POWER|BUTTON_REPEAT,         BUTTON_POWER },
    { ACTION_TREE_STOP,    BUTTON_PLAY|BUTTON_REPEAT,         BUTTON_PLAY },
    { ACTION_TREE_WPS,   BUTTON_POWER|BUTTON_REL,      BUTTON_POWER },
    { ACTION_TREE_WPS,   BUTTON_PLAY|BUTTON_REL,      BUTTON_PLAY },
    { ACTION_STD_MENU,   BUTTON_LEFT|BUTTON_REPEAT,      BUTTON_LEFT },
    //TODO: { ACTION_TREE_HOTKEY, BUTTON_REC|BUTTON_REL,        BUTTON_REC },

    { ACTION_TREE_PGLEFT,   BUTTON_RIGHT|BUTTON_UP,               BUTTON_NONE },
    { ACTION_TREE_ROOT_INIT,BUTTON_RIGHT|BUTTON_UP|BUTTON_REPEAT, BUTTON_RIGHT|BUTTON_UP },
    { ACTION_TREE_PGLEFT,   BUTTON_RIGHT|BUTTON_UP|BUTTON_REPEAT, BUTTON_NONE },
    { ACTION_TREE_PGRIGHT,  BUTTON_RIGHT|BUTTON_DOWN,              BUTTON_NONE },
    { ACTION_TREE_PGRIGHT,  BUTTON_RIGHT|BUTTON_DOWN|BUTTON_REPEAT,BUTTON_NONE },

    LAST_ITEM_IN_LIST__NEXTLIST(CONTEXT_LIST),
}; /* button_context_tree */

static const struct button_mapping button_context_yesno[]  = {

    { ACTION_YESNO_ACCEPT,          BUTTON_RIGHT|BUTTON_REL,              BUTTON_RIGHT },
    { ACTION_YESNO_ACCEPT,          BUTTON_PLAY|BUTTON_REL,              BUTTON_PLAY },

    LAST_ITEM_IN_LIST__NEXTLIST(CONTEXT_STD),
}; /* button_context_settings_yesno */

static const struct button_mapping button_context_quickscreen[]  = {
    // { ACTION_STD_CANCEL, BUTTON_POWER|BUTTON_REL,       BUTTON_POWER },
    // { ACTION_STD_CANCEL, BUTTON_PLAY|BUTTON_REL,       BUTTON_PLAY },
    { ACTION_QS_SHORTCUTS, BUTTON_POWER|BUTTON_REPEAT,       BUTTON_POWER },
    { ACTION_QS_SHORTCUTS, BUTTON_PLAY|BUTTON_REPEAT,       BUTTON_PLAY },
    { ACTION_QS_TOP,     BUTTON_UP|BUTTON_REL,          BUTTON_UP },
    { ACTION_QS_TOP,     BUTTON_UP|BUTTON_REPEAT,       BUTTON_NONE },
    { ACTION_QS_DOWN,    BUTTON_DOWN|BUTTON_REL,        BUTTON_DOWN },
    { ACTION_QS_DOWN,    BUTTON_DOWN|BUTTON_REPEAT,     BUTTON_NONE },
    { ACTION_QS_LEFT,    BUTTON_LEFT|BUTTON_REL,        BUTTON_LEFT },
    { ACTION_QS_LEFT,    BUTTON_LEFT|BUTTON_REPEAT,     BUTTON_NONE },
    { ACTION_QS_RIGHT,   BUTTON_RIGHT|BUTTON_REL,       BUTTON_RIGHT },
    { ACTION_QS_RIGHT,   BUTTON_RIGHT|BUTTON_REPEAT,    BUTTON_NONE },

    { ACTION_QS_LEFT,    BUTTON_PREV|BUTTON_REL,             BUTTON_PREV },
    { ACTION_QS_LEFT,    BUTTON_PREV|BUTTON_REPEAT,          BUTTON_NONE },
    { ACTION_QS_RIGHT,   BUTTON_NEXT|BUTTON_REL,             BUTTON_NEXT },
    { ACTION_QS_RIGHT,   BUTTON_NEXT|BUTTON_REPEAT,          BUTTON_NONE },
    
    LAST_ITEM_IN_LIST__NEXTLIST(CONTEXT_STD),
}; /* button_context_quickscreen */

static const struct button_mapping button_context_settings_right_is_inc[]  = {
        { ACTION_SETTINGS_INC,      BUTTON_RIGHT,               BUTTON_NONE },
        { ACTION_NONE,      BUTTON_RIGHT|BUTTON_REL,               BUTTON_RIGHT },
        { ACTION_SETTINGS_INCREPEAT,BUTTON_RIGHT|BUTTON_REPEAT, BUTTON_NONE },
        { ACTION_SETTINGS_DEC,      BUTTON_LEFT,                BUTTON_NONE },
        { ACTION_NONE,      BUTTON_LEFT|BUTTON_REL,                BUTTON_LEFT },
        { ACTION_SETTINGS_DECREPEAT,BUTTON_LEFT|BUTTON_REPEAT,  BUTTON_NONE },
        { ACTION_STD_PREV,                  BUTTON_UP,                         BUTTON_NONE },
        { ACTION_STD_PREVREPEAT,            BUTTON_UP|BUTTON_REPEAT,           BUTTON_NONE },
        { ACTION_STD_NEXT,                  BUTTON_DOWN,                       BUTTON_NONE },
        { ACTION_STD_NEXTREPEAT,            BUTTON_DOWN|BUTTON_REPEAT,         BUTTON_NONE },
        { ACTION_STD_OK,            BUTTON_POWER|BUTTON_REL,         BUTTON_POWER },
        { ACTION_STD_MENU,            BUTTON_POWER|BUTTON_REPEAT,         BUTTON_POWER },
        { ACTION_STD_CANCEL,            BUTTON_POWER|BUTTON_LEFT,         BUTTON_NONE },
        { ACTION_STD_OK,            BUTTON_POWER|BUTTON_RIGHT,         BUTTON_NONE },
        LAST_ITEM_IN_LIST__NEXTLIST(CONTEXT_SETTINGS),
}; /* button_context_settings_right_is_inc */

static const struct button_mapping button_context_pitchscreen[]  = {
        { ACTION_PS_INC_SMALL,      BUTTON_UP,                      BUTTON_NONE },
        { ACTION_PS_INC_BIG,        BUTTON_UP|BUTTON_REPEAT,        BUTTON_NONE },
        { ACTION_PS_DEC_SMALL,      BUTTON_DOWN,                    BUTTON_NONE },
        { ACTION_PS_DEC_BIG,        BUTTON_DOWN|BUTTON_REPEAT,      BUTTON_NONE },
        { ACTION_PS_NUDGE_LEFT,     BUTTON_LEFT,                    BUTTON_NONE },
        { ACTION_PS_NUDGE_LEFTOFF,  BUTTON_LEFT|BUTTON_REL,         BUTTON_NONE },
        { ACTION_PS_NUDGE_RIGHT,    BUTTON_RIGHT,                   BUTTON_NONE },
        { ACTION_PS_NUDGE_RIGHTOFF, BUTTON_RIGHT|BUTTON_REL,        BUTTON_NONE },
        { ACTION_PS_TOGGLE_MODE,    BUTTON_POWER|BUTTON_REPEAT,       BUTTON_POWER },
        { ACTION_PS_RESET,          BUTTON_POWER|BUTTON_DOWN,    BUTTON_NONE },
        { ACTION_PS_RESET,          BUTTON_POWER|BUTTON_LEFT,    BUTTON_NONE },
        { ACTION_PS_EXIT,           BUTTON_POWER|BUTTON_REL,         BUTTON_POWER },
        { ACTION_PS_SLOWER,         BUTTON_LEFT|BUTTON_REPEAT,      BUTTON_NONE },
        { ACTION_PS_FASTER,         BUTTON_RIGHT|BUTTON_REPEAT,     BUTTON_NONE },
        LAST_ITEM_IN_LIST__NEXTLIST(CONTEXT_STD),
}; /* button_context_pitchscreen */


static const struct button_mapping button_context_keyboard[]  = {
    { ACTION_KBD_LEFT,         BUTTON_LEFT,                      BUTTON_NONE },
    { ACTION_KBD_LEFT,         BUTTON_LEFT|BUTTON_REPEAT,        BUTTON_NONE },
    { ACTION_KBD_RIGHT,        BUTTON_RIGHT,                     BUTTON_NONE },
    { ACTION_KBD_RIGHT,        BUTTON_RIGHT|BUTTON_REPEAT,       BUTTON_NONE },

    // { ACTION_KBD_CURSOR_LEFT,  BUTTON_REC|BUTTON_LEFT,                BUTTON_NONE },
    // { ACTION_KBD_CURSOR_LEFT,  BUTTON_REC|BUTTON_LEFT|BUTTON_REPEAT,  BUTTON_NONE },
    // { ACTION_KBD_CURSOR_RIGHT, BUTTON_REC|BUTTON_RIGHT,               BUTTON_NONE },
    // { ACTION_KBD_CURSOR_RIGHT, BUTTON_REC|BUTTON_RIGHT|BUTTON_REPEAT, BUTTON_NONE },
    //
    { ACTION_KBD_UP,           BUTTON_UP,                 BUTTON_NONE },
    { ACTION_KBD_UP,           BUTTON_UP|BUTTON_REPEAT,   BUTTON_NONE },
    { ACTION_KBD_DOWN,         BUTTON_DOWN,               BUTTON_NONE },
    { ACTION_KBD_DOWN,         BUTTON_DOWN|BUTTON_REPEAT, BUTTON_NONE },
    //TODO: { ACTION_KBD_PAGE_FLIP,    BUTTON_REC|BUTTON_SELECT,         BUTTON_REC },
    { ACTION_KBD_BACKSPACE,    BUTTON_POWER|BUTTON_DOWN,         BUTTON_NONE },
    { ACTION_KBD_SELECT,       BUTTON_POWER|BUTTON_REL,                    BUTTON_POWER },
    { ACTION_KBD_DONE,         BUTTON_POWER|BUTTON_RIGHT,                        BUTTON_NONE },
    { ACTION_KBD_ABORT,        BUTTON_POWER|BUTTON_LEFT,                     BUTTON_NONE },
    // { ACTION_KBD_MORSE_INPUT,  BUTTON_POWER|BUTTON_UP,            BUTTON_POWER },
    // { ACTION_KBD_MORSE_SELECT, BUTTON_POWER|BUTTON_REL,         BUTTON_POWER },

    LAST_ITEM_IN_LIST
}; /* button_context_keyboard */

static const struct button_mapping button_context_bmark[]  = {
    // { ACTION_BMS_DELETE,       BUTTON_REC,        BUTTON_NONE },

    LAST_ITEM_IN_LIST__NEXTLIST(CONTEXT_LIST),
}; /* button_context_bmark */

/* get_context_mapping returns a pointer to one of the above defined arrays depending on the context */
const struct button_mapping* target_get_context_mapping(int context)
{
    switch (context)
    {
        case CONTEXT_STD:
            return button_context_standard;

        case CONTEXT_WPS | CONTEXT_LOCKED:
            return button_context_wps_locked;
        case CONTEXT_WPS:
            return button_context_wps;

        case CONTEXT_LIST:
        case CONTEXT_LIST | CONTEXT_LOCKED:
            return button_context_list;
        case CONTEXT_TREE | CONTEXT_LOCKED:
        case CONTEXT_TREE:
        case CONTEXT_CUSTOM|CONTEXT_TREE:
            return button_context_tree;

        case CONTEXT_MAINMENU|CONTEXT_LOCKED:
        case CONTEXT_MAINMENU:
            return button_goto_tree_context;

        case CONTEXT_SETTINGS:
            return button_context_settings;

        case CONTEXT_SETTINGS_TIME:
            return button_goto_settings_context;

        case CONTEXT_SETTINGS_COLOURCHOOSER:
        case CONTEXT_SETTINGS_EQ:
            return button_goto_settings_right_is_inc_context;
        //case CONTEXT_SETTINGS_RECTRIGGER:
        case CONTEXT_CUSTOM|CONTEXT_SETTINGS:
            return button_context_settings_right_is_inc;

        case CONTEXT_YESNOSCREEN:
            return button_context_yesno;

        case CONTEXT_BOOKMARKSCREEN:
            return button_context_bmark;
        case CONTEXT_QUICKSCREEN:
            return button_context_quickscreen;
        case CONTEXT_PITCHSCREEN:
            return button_context_pitchscreen;
        case CONTEXT_KEYBOARD:
//        case CONTEXT_MORSE_INPUT:
            return button_context_keyboard;
    } 
    return button_goto_std_context;
}