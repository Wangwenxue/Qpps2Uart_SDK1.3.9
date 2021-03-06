/**
 ****************************************************************************************
 *
 * @file htpt_task.c
 *
 * @brief Health Thermometer Profile Thermometer Task implementation.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup HTPTTASK
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_config.h"

#if (BLE_HT_THERMOM)

#include "gap.h"
#include "gatt_task.h"
#include "atts_util.h"
#include "htpt.h"
#include "htpt_task.h"
#include "prf_utils.h"

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

static int htpt_create_db_req_handler(ke_msg_id_t const msgid,
                                      struct htpt_create_db_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    //Valid Range value
    uint32_t valid_range;
    //Service content flag
    uint16_t cfg_flag;

    uint16_t total_size = 0;
    uint8_t nb_att = 0;
    uint8_t status = ATT_ERR_NO_ERROR;
    uint8_t i;

    // Save profile ID
    htpt_env.con_info.prf_id = TASK_HTPT;

    //Save database configuration
    htpt_env.features = param->features;

    //Compute Attribute Table and save it in environment
    cfg_flag = htpt_compute_att_table(param->features);

    status = atts_svc_create_db(&htpt_env.shdl, (uint8_t *)&cfg_flag, HTS_IDX_NB, &htpt_env.att_tbl[0],
                               dest_id, &htpt_att_db[0]);

    //Disable the service and set optional features
    if (status == ATT_ERR_NO_ERROR)
    {
        //Disable service
        status = attsdb_svc_set_permission(htpt_env.shdl, PERM(SVC, DISABLE));

        //Set optional properties and permissions
        if (htpt_env.att_tbl[HTPT_MEAS_INTV_CHAR] != 0x00)
        {
            nb_att = ATT_CHAR_PROP_RD;
            total_size = PERM(RD, ENABLE);
            i = 1;

            if ((param->features & HTPT_MEAS_INTV_IND_SUP) == HTPT_MEAS_INTV_IND_SUP)
            {
                nb_att |= ATT_CHAR_PROP_IND;
                total_size |= PERM(IND, ENABLE);
                i++;
            }

            if ((param->features & HTPT_MEAS_INTV_WR_SUP) == HTPT_MEAS_INTV_WR_SUP)
            {
                nb_att |= ATT_CHAR_PROP_WR;
                total_size |= PERM(WR, UNAUTH);
                i++;

                //Set Measurement Interval Char. - Valid Range value
                if ((param->valid_range_min) < (param->valid_range_max))
                {
                    valid_range = param->valid_range_min | (param->valid_range_max << 16);
                }
                else
                {
                    //If minimal and maximal values are the same or if maximal value is upper than minimal value
                    valid_range = HTPT_MEAS_INTV_DFLT_MIN | (HTPT_MEAS_INTV_DFLT_MAX << 16);
                }

                status = attsdb_att_set_value(htpt_env.shdl + htpt_env.att_tbl[HTPT_MEAS_INTV_CHAR] + i,
                                              sizeof(uint32_t), (uint8_t *)&valid_range);

            }

            attsdb_att_partial_value_update(htpt_env.shdl + htpt_env.att_tbl[HTPT_MEAS_INTV_CHAR], 0, 1, &nb_att);
            attsdb_att_set_permission(htpt_env.shdl + htpt_env.att_tbl[HTPT_MEAS_INTV_CHAR] + 1, total_size);
        }

        //If we are here, database has been fulfilled with success, go to idle test
        ke_state_set(TASK_HTPT, HTPT_IDLE);
    }

    //Send response to application
    struct htpt_create_db_cfm * cfm = KE_MSG_ALLOC(HTPT_CREATE_DB_CFM, src_id, TASK_HTPT,
                                                   htpt_create_db_cfm);
    cfm->status = status;
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref HTPT_ENABLE_REQ message.
 * The handler enables the Health Thermometer Profile Thermometer Role.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int htpt_enable_req_handler(ke_msg_id_t const msgid,
                                   struct htpt_enable_req const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    uint16_t value = 0;

    //Save the application task id
    htpt_env.con_info.appid = src_id;
    //Save the connection handle associated to the profile
    htpt_env.con_info.conhdl = param->conhdl;

    // Check if the provided connection exist
    if (gap_get_rec_idx(param->conhdl) == GAP_INVALID_CONIDX)
    {
        // The connection doesn't exist, request disallowed
        prf_server_error_ind_send((prf_env_struct *)&htpt_env, PRF_ERR_REQ_DISALLOWED,
                                  HTPT_ERROR_IND, HTPT_ENABLE_REQ);
    }
    else
    {
        //Configure Intermediate Temp Ntf Cfg in DB
        if (htpt_env.att_tbl[HTPT_INTERM_TEMP_CHAR] != 0x00)
        {
            //Written value is 0 is discovery connection, given value if normal
            if(param->con_type == PRF_CON_NORMAL)
            {
                memcpy(&value, &param->interm_temp_ntf_en, sizeof(uint16_t));

                if (param->interm_temp_ntf_en == PRF_CLI_START_NTF)
                {
                    htpt_env.features |= HTPT_MASK_INTM_MEAS_CFG;
                }
            }

            attsdb_att_set_value(htpt_env.shdl + htpt_env.att_tbl[HTPT_INTERM_TEMP_CHAR] + 2, sizeof(uint16_t),
                                 (uint8_t *)&value);

            //Reset value
            value = 0;
        }

        //Configure Meas. Interval value and IND Cfg in DB
        if (htpt_env.att_tbl[HTPT_MEAS_INTV_CHAR] != 0x00)
        {
            //----------- Meas. Intv. Value ----------------------------------------------
            //Written value is 0 is discovery connection, given value if normal
            if(param->con_type == PRF_CON_NORMAL)
            {
                memcpy(&value, &param->meas_intv, sizeof(uint16_t));
            }

            attsdb_att_set_value(htpt_env.shdl + htpt_env.att_tbl[HTPT_MEAS_INTV_CHAR] + 1, sizeof(uint16_t),
                                 (uint8_t *)&value);

            //Reset value
            value = 0;

            //----------- IND Cfg ----------------------------------------------
            if ((htpt_env.features & HTPT_MEAS_INTV_IND_SUP) == HTPT_MEAS_INTV_IND_SUP)
            {
                //Written value is 0 is discovery connection, given value if normal
                if(param->con_type == PRF_CON_NORMAL)
                {
                    memcpy(&value, &param->meas_intv_ind_en, sizeof(uint16_t));

                    if (param->meas_intv_ind_en == PRF_CLI_START_IND)
                    {
                        htpt_env.features |= HTPT_MASK_MEAS_INTV_CFG;
                    }
                }

                attsdb_att_set_value(htpt_env.shdl + htpt_env.att_tbl[HTPT_MEAS_INTV_CHAR] + 2, sizeof(uint16_t),
                                     (uint8_t *)&value);

                //Reset value
                value = 0;
            }
        }

        //Configure Temp Meas value IND Cfg in DB
        if(param->con_type == PRF_CON_NORMAL)
        {
            memcpy(&value, &param->temp_meas_ind_en, sizeof(uint16_t));

            if (param->temp_meas_ind_en == PRF_CLI_START_IND)
            {
                htpt_env.features |= HTPT_MASK_TEMP_MEAS_CFG;
            }
        }

        attsdb_att_set_value(htpt_env.shdl + HTS_IDX_TEMP_MEAS_IND_CFG, sizeof(uint16_t),
                             (uint8_t *)&value);

        //Reset value
        value = 1;

        //Initialize Temp Type value in DB
        if (htpt_env.att_tbl[HTPT_TEMP_TYPE_CHAR] != 0x00)
        {
            attsdb_att_set_value(htpt_env.shdl + HTS_IDX_TEMP_TYPE_VAL, sizeof(uint8_t),
                                 (uint8_t *)&value);
        }


        //Enable Attributes + Set Security Level
        attsdb_svc_set_permission(htpt_env.shdl, param->sec_lvl);

        // Go to connected state
        ke_state_set(TASK_HTPT, HTPT_CONNECTED);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref HTPT_TEMP_UPD_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int htpt_temp_send_req_handler(ke_msg_id_t const msgid,
                                      struct htpt_temp_send_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    // Packed Temperature Measurement value
    uint8_t packed_temp[HTPT_TEMP_MEAS_MAX_LEN];
    // Status
    uint8_t status = PRF_ERR_OK;
    // Packet size
    uint8_t size;

    if(param->conhdl == htpt_env.con_info.conhdl)
    {
        //Pack the temperature measurement value
        size = htpt_pack_temp_value(&packed_temp[0], param->temp_meas);

        //Intermediate Measurement, must be notified if enabled
        if(param->flag_stable_meas == 0x00)
        {
            //Check if Intermediate Measurement Char. is supported
            if (htpt_env.att_tbl[HTPT_INTERM_TEMP_CHAR] != 0x00)
            {
                //Check if notifications are enabled
                if((htpt_env.features & HTPT_MASK_INTM_MEAS_CFG) == HTPT_MASK_INTM_MEAS_CFG)
                {
                    attsdb_att_set_value(htpt_env.shdl + htpt_env.att_tbl[HTPT_INTERM_TEMP_CHAR] + 1,
                                         size, &packed_temp[0]);

                    //Send notification through GATT
                    struct gatt_notify_req * ntf = KE_MSG_ALLOC(GATT_NOTIFY_REQ, TASK_GATT,
                                                                TASK_HTPT, gatt_notify_req);

                    ntf->conhdl  = htpt_env.con_info.conhdl;
                    ntf->charhdl = htpt_env.shdl + htpt_env.att_tbl[HTPT_INTERM_TEMP_CHAR] + 1;

                    ke_msg_send(ntf);
                }
                else
                {
                    status = PRF_ERR_NTF_DISABLED;
                }
            }
            else
            {
                status = PRF_ERR_FEATURE_NOT_SUPPORTED;
            }
        }
        //Temperature Measurement, must be indicated if enabled
        else
        {
            if((htpt_env.features & HTPT_MASK_TEMP_MEAS_CFG) == HTPT_MASK_TEMP_MEAS_CFG)
            {
                attsdb_att_set_value(htpt_env.shdl + HTS_IDX_TEMP_MEAS_VAL, size, &packed_temp[0]);

                struct gatt_indicate_req *ind = KE_MSG_ALLOC(GATT_INDICATE_REQ, TASK_GATT,
                                                             TASK_HTPT, gatt_indicate_req);

                ind->conhdl  = htpt_env.con_info.conhdl;
                ind->charhdl = htpt_env.shdl + HTS_IDX_TEMP_MEAS_VAL;

                ke_msg_send(ind);
            }
            else
            {
                status = PRF_ERR_IND_DISABLED;
            }
        }
    }
    else
    {
        //Wrong Connection Handle
        status = PRF_ERR_INVALID_PARAM;
    }

    if (status != PRF_ERR_OK)
    {
        /*
         * The cfm_type value is HTPT_CENTRAL_IND_CFM (1) if param->flag_stable_meas = 1,
         * HTPT_THERM_TEMP_SEND (0) if param->flag_stable_meas = 0
         */
        htpt_temp_send_cfm_send(status, param->flag_stable_meas);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Request to update Measurement Interval Value
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int htpt_meas_intv_upd_req_handler(ke_msg_id_t const msgid,
                                          struct htpt_meas_intv_upd_req const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    uint16_t handle;

    //Check if Measurement Interval is supported
    if (htpt_env.att_tbl[HTPT_MEAS_INTV_CHAR] != 0x00)
    {
        //Check Connection Handle
        if(param->conhdl == htpt_env.con_info.conhdl)
        {
            handle = htpt_env.shdl + htpt_env.att_tbl[HTPT_MEAS_INTV_CHAR] + 1;

            //Update saved value in database
            attsdb_att_set_value(handle, sizeof(param->meas_intv), (uint8_t *)&param->meas_intv);

            //Must be indicated if enabled
            if((htpt_env.features & HTPT_MASK_MEAS_INTV_CFG) == HTPT_MASK_MEAS_INTV_CFG)
            {
                struct gatt_indicate_req * ind = KE_MSG_ALLOC(GATT_INDICATE_REQ, TASK_GATT,
                                                              TASK_HTPT, gatt_indicate_req);

                ind->conhdl  = htpt_env.con_info.conhdl;
                ind->charhdl = handle;

                ke_msg_send(ind);
            }
        }
        else
        {
            //Wrong Connection Handle
            prf_server_error_ind_send((prf_env_struct *)&htpt_env, PRF_ERR_INVALID_PARAM,
                                      HTPT_ERROR_IND, HTPT_MEAS_INTV_UPD_REQ);
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref HTPT_TEMP_TYPE_UPD_REQ message.
 * The handler compares the new values with current ones and notifies them if they changed.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int htpt_temp_type_upd_req_handler(ke_msg_id_t const msgid,
                                          struct htpt_temp_type_upd_req const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    if (htpt_env.att_tbl[HTPT_TEMP_TYPE_CHAR] != 0x00)
    {
        //Update value in database
        attsdb_att_set_value(htpt_env.shdl + htpt_env.att_tbl[HTPT_TEMP_TYPE_CHAR] + 1,
                             sizeof(param->value), (uint8_t *)&(param->value));
    }
    else
    {
        prf_server_error_ind_send((prf_env_struct *)&htpt_env, PRF_ERR_FEATURE_NOT_SUPPORTED,
                                  HTPT_ERROR_IND, HTPT_TEMP_TYPE_UPD_REQ);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_WRITE_CMD_IND message.
 * The handler compares the new values with current ones and notifies them if they changed.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_write_cmd_ind_handler(ke_msg_id_t const msgid,
                                      struct gatt_write_cmd_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    uint8_t *meas_intv_range;
    uint16_t meas_intv_rge[2];

    uint16_t value = 0x0000;
    uint8_t status = PRF_ERR_OK;
    uint8_t char_code = 0;

    if (param->conhdl == htpt_env.con_info.conhdl)
    {
        //Extract value before check
        memcpy(&value, &(param->value), sizeof(uint16_t));

        //Measurement Interval Char. - Value
        if (param->handle == (htpt_env.shdl + htpt_env.att_tbl[HTPT_MEAS_INTV_CHAR] + 1))
        {
            /*
             * Get Measurement Interval range in database
             * Valid Range descriptor exists because measurement interval is writable
             */
            attsdb_att_get_value(param->handle + htpt_get_valid_rge_offset(), &meas_intv_rge[0], &meas_intv_range);

            memcpy(&meas_intv_rge[0], meas_intv_range, sizeof(uint16_t));
            memcpy(&meas_intv_rge[1], meas_intv_range + 2, sizeof(uint16_t));

            //Check if value to write is in allowed range
            if (((value >= meas_intv_rge[0]) && (value <= meas_intv_rge[1])) || (value == 0))
            {
                if(param->last)
                {
                    //Send APP the indication with the new value
                    struct htpt_meas_intv_chg_ind * ind = KE_MSG_ALLOC(HTPT_MEAS_INTV_CHG_IND,
                                                                       htpt_env.con_info.appid, TASK_HTPT,
                                                                       htpt_meas_intv_chg_ind);

                    memcpy(&ind->intv, &value, sizeof(uint16_t));

                    ke_msg_send(ind);
                }
            }
            else
            {
                status = HTPT_OUT_OF_RANGE_ERR_CODE;
            }
        }
        else
        {
            //Temperature Measurement Char. - Client Char. Configuration
            if (param->handle == (htpt_env.shdl + HTS_IDX_TEMP_MEAS_IND_CFG))
            {
                char_code = HTPT_TEMP_MEAS_CHAR;

                if (value == PRF_CLI_STOP_NTFIND)
                {
                    htpt_env.features &= ~HTPT_MASK_TEMP_MEAS_CFG;
                }
                else if (value == PRF_CLI_START_IND)
                {
                    htpt_env.features |= HTPT_MASK_TEMP_MEAS_CFG;
                }
                else
                {
                    //Invalid value
                    status = HTPT_OUT_OF_RANGE_ERR_CODE;
                }
            }
            //Measurement Interval Char. - Client Char. Configuration
            else if (param->handle == (htpt_env.shdl + htpt_env.att_tbl[HTPT_MEAS_INTV_CHAR] + 2))
            {
                char_code = HTPT_MEAS_INTV_CHAR;

                if (value == PRF_CLI_STOP_NTFIND)
                {
                    htpt_env.features &= ~HTPT_MASK_MEAS_INTV_CFG;
                }
                else if (value == PRF_CLI_START_IND)
                {
                    htpt_env.features |= HTPT_MASK_MEAS_INTV_CFG;
                }
                else
                {
                    //Invalid value
                    status = HTPT_OUT_OF_RANGE_ERR_CODE;
                }
            }
            //Intermediate Measurement Char. - Client Char. Configuration
            else if (param->handle == (htpt_env.shdl + htpt_env.att_tbl[HTPT_INTERM_TEMP_CHAR] + 2))
            {
                char_code = HTPT_INTERM_TEMP_CHAR;

                if (value == PRF_CLI_STOP_NTFIND)
                {
                    htpt_env.features &= ~HTPT_MASK_INTM_MEAS_CFG;
                }
                else if (value == PRF_CLI_START_NTF)
                {
                    htpt_env.features |= HTPT_MASK_INTM_MEAS_CFG;
                }
                else
                {
                    //Invalid value
                    status = HTPT_OUT_OF_RANGE_ERR_CODE;
                }
            }

            if (status == PRF_ERR_OK)
            {
                if(param->last)
                {
                    //Inform APP of configuration change
                    struct htpt_cfg_indntf_ind * ind = KE_MSG_ALLOC(HTPT_CFG_INDNTF_IND,
                                                                    htpt_env.con_info.appid, TASK_HTPT,
                                                                    htpt_cfg_indntf_ind);

                    memcpy(&ind->conhdl, &htpt_env.con_info.conhdl, sizeof(uint16_t));
                    ind->char_code = char_code;
                    memcpy(&ind->cfg_val, &value, sizeof(uint16_t));

                    ke_msg_send(ind);
                }
            }
        }

        if (status == PRF_ERR_OK)
        {
            //Update the attribute value
            attsdb_att_set_value(param->handle, sizeof(uint16_t), (uint8_t *)&value);
        }

        //Send write response
        atts_write_rsp_send(htpt_env.con_info.conhdl, param->handle, status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles @ref GATT_HANDLE_VALUE_CFM message meaning that Measurement indication
 * has been correctly sent to peer device.
 *
 * Convey this information to appli task using @ref HTPT_TEMP_SEND_CFM
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_handle_value_cfm_handler(ke_msg_id_t const msgid,
                                         struct gatt_handle_value_cfm const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
    //Send confirmation to APP that value was sent/not
    htpt_temp_send_cfm_send(param->status, HTPT_CENTRAL_IND_CFM);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles @ref GATT_NOTIFY_CMP_EVT message meaning that Measurement notification
 * has been correctly sent to peer device (but not confirmed by peer device).
 *
 * Convey this information to appli task using @ref HTPT_MEAS_SEND_CFM
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_notify_cmp_evt_handler(ke_msg_id_t const msgid,
                                       struct gatt_notify_cmp_evt const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
#if (QN_MULTI_NOTIFICATION_IN_ONE_EVENT)  
    if(param->status == GATT_NOTIFY_GET_DATA)
        return (KE_MSG_CONSUMED);        
#endif

    htpt_temp_send_cfm_send(param->status, HTPT_THERM_TEMP_SEND);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Disconnection indication to HTPT.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gap_discon_cmp_evt_handler(ke_msg_id_t const msgid,
                                      struct gap_discon_cmp_evt const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    if (param->conhdl == htpt_env.con_info.conhdl)
    {
        htpt_disable();
    }

    return (KE_MSG_CONSUMED);
}


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

///Disabled State handler definition.
const struct ke_msg_handler htpt_disabled[] =
{
    {HTPT_CREATE_DB_REQ,         (ke_msg_func_t)htpt_create_db_req_handler}
};

///Idle State handler definition.
const struct ke_msg_handler htpt_idle[] =
{
    {HTPT_ENABLE_REQ,            (ke_msg_func_t)htpt_enable_req_handler},
    {HTPT_TEMP_TYPE_UPD_REQ,     (ke_msg_func_t)htpt_temp_type_upd_req_handler},
};

///Idle State handler definition.
const struct ke_msg_handler htpt_connected[] =
{
    {HTPT_TEMP_SEND_REQ,         (ke_msg_func_t)htpt_temp_send_req_handler},
    {HTPT_MEAS_INTV_UPD_REQ,     (ke_msg_func_t)htpt_meas_intv_upd_req_handler},
    {HTPT_TEMP_TYPE_UPD_REQ,     (ke_msg_func_t)htpt_temp_type_upd_req_handler},
    {GATT_HANDLE_VALUE_CFM,      (ke_msg_func_t)gatt_handle_value_cfm_handler},
    {GATT_NOTIFY_CMP_EVT,        (ke_msg_func_t)gatt_notify_cmp_evt_handler},
    {GATT_WRITE_CMD_IND,         (ke_msg_func_t)gatt_write_cmd_ind_handler},
};

/// Default State handlers definition
const struct ke_msg_handler htpt_default_state[] =
{
    {GAP_DISCON_CMP_EVT,         (ke_msg_func_t)gap_discon_cmp_evt_handler},
};

///Specifies the message handler structure for every input state.
const struct ke_state_handler htpt_state_handler[HTPT_STATE_MAX] =
{
    [HTPT_DISABLED]    = KE_STATE_HANDLER(htpt_disabled),
    [HTPT_IDLE]        = KE_STATE_HANDLER(htpt_idle),
    [HTPT_CONNECTED]   = KE_STATE_HANDLER(htpt_connected),
};

///Specifies the message handlers that are common to all states.
const struct ke_state_handler htpt_default_handler = KE_STATE_HANDLER(htpt_default_state);

///Defines the place holder for the states of all the task instances.
ke_state_t htpt_state[HTPT_IDX_MAX];

// Register HTPT task into kernel
void task_htpt_desc_register(void)
{
    struct ke_task_desc task_htpt_desc;
    
    task_htpt_desc.state_handler = htpt_state_handler;
    task_htpt_desc.default_handler=&htpt_default_handler;
    task_htpt_desc.state = htpt_state;
    task_htpt_desc.state_max = HTPT_STATE_MAX;
    task_htpt_desc.idx_max = HTPT_IDX_MAX;

    task_desc_register(TASK_HTPT , task_htpt_desc);
}

#endif //BLE_H T_THERMOM

/// @} HTPTTASK
