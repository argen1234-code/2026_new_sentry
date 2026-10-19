#include "gimbal_channel.h"
#include "gimbal_config.h"
#include "auto_aim.h"
#include <string.h>

uint16_t aaa = 0;
uint8_t bbb = 0;
int test =0;
/**
  * @brief                  浜戝彴閫氶亾RX鍥炶皟锛堝鐞嗘帶鍒跺懡浠わ級
  * @param[in] ch           閫氶亾瀵硅薄锛堟湭浣跨敤锛?
  * @param[in] payload      鎺ユ敹鏁版嵁缂撳啿鍖?
  * @param[in] len          鏁版嵁闀垮害锛堝瓧鑺傦級
  * @param[in] user         鐢ㄦ埛涓婁笅鏂囷紙gimbal_channel_t鎸囬拡锛?
  * @retval                 none
  */
static void on_rx(channel_t *ch, const uint8_t *payload, uint32_t len, void *user) {
    (void)ch;
    gimbal_channel_t *gc = (gimbal_channel_t *)user;
    if(!gc || !payload || len < 2)
        return;
    uint16_t sid = comm_read_u16_le(payload);
    if(sid == GIMBAL_SID_DELTA) {
        if(len < 2 + 4 + 4 + 2 + 8 + 4 + 4 + 4 + 2)
            return;
        gimbal_delta_t d;
        d.delta_yaw_udeg = comm_read_i32_le(&payload[2]);
        d.delta_pitch_udeg = comm_read_i32_le(&payload[6]);
        d.status = comm_read_u16_le(&payload[10]);
				aaa = d.status;
        d.ts_us = comm_read_u64_le(&payload[12]);
				d.vx = comm_read_i32_le(&payload[20]);
        d.vy = -comm_read_i32_le(&payload[24]);
 				d.wz = -comm_read_i32_le(&payload[28]);
        d.flag_wz = comm_read_u16_le(&payload[32]);
				test = d.flag_wz;
				//更新数据
				bbb = d.flag_wz;
				auto_aim_apply_delta_udeg(d.delta_yaw_udeg, d.delta_pitch_udeg, d.status,  d.ts_us, d.vx, d.vy, d.wz, d.flag_wz);
        if(gc->hooks.on_delta) gc->hooks.on_delta(&d, gc->hooks.user);
#if defined(GIMBAL_DELTA_ACK_ENABLE) && (GIMBAL_DELTA_ACK_ENABLE)
        uint8_t ack[2];
        comm_write_u16_le(ack, GIMBAL_SID_DELTA);
        (void)ch_uproto_queue_notify(gc->bind, gc->ch_id, GIMBAL_SID_DELTA, 0, ack, 2);
#endif
        return;
    }

    if(sid == GIMBAL_SID_BIG_YAW_DELTA) {
        if(len < 2 + 4 + 2 + 8)
            return;
        gimbal_big_yaw_delta_t d;
        d.delta_yaw_udeg = comm_read_i32_le(&payload[2]);
        d.status = comm_read_u16_le(&payload[6]);
        d.ts_us = comm_read_u64_le(&payload[8]);
        auto_aim_apply_big_yaw_delta_udeg(d.delta_yaw_udeg, d.status, d.ts_us);
        if(gc->hooks.on_big_yaw_delta) gc->hooks.on_big_yaw_delta(&d, gc->hooks.user);
				
#if defined(GIMBAL_DELTA_ACK_ENABLE) && (GIMBAL_DELTA_ACK_ENABLE)
        uint8_t ack[2];
        comm_write_u16_le(ack, GIMBAL_SID_BIG_YAW_DELTA);
        (void)ch_uproto_queue_notify(gc->bind, gc->ch_id, GIMBAL_SID_BIG_YAW_DELTA, 0, ack, 2);
#endif
        return;
    }
}

/**
  * @brief                  浜戝彴閫氶亾瀹氭椂鍥炶皟锛堝懆鏈熸€у彂甯冪姸鎬侊級
  * @param[in] ch           閫氶亾瀵硅薄锛堟湭浣跨敤锛?
  * @param[in] user         鐢ㄦ埛涓婁笅鏂囷紙gimbal_channel_t鎸囬拡锛?
  * @retval                 none
  */
static void on_tick(channel_t *ch, void *user) {
    (void)ch;
    gimbal_channel_t *gc = (gimbal_channel_t *)user;
    if(!gc)
        return;
    if(!gc->src.get_state || gc->src.period_ms == 0)
        return;
    uint64_t now = gc->now_us ? gc->now_us(gc->time_user) : 0;
    if(gc->last_pub_us == 0 || (now - gc->last_pub_us) >= (uint64_t)gc->src.period_ms * 1000ULL) {
        (void)gimbal_channel_publish(gc, NULL);
        gc->last_pub_us = now;
    }
}

/**
  * @brief                  鍒濆鍖栦簯鍙伴€氶亾锛堝吋瀹圭増鍙傛暟锛?
  * @param[in] gc           浜戝彴閫氶亾瀹炰緥
  * @param[in] bind         uproto浼犺緭缁戝畾涓婁笅鏂?
  * @param[in] mgr          鎵€灞為€氶亾绠＄悊鍣?
  * @param[in] ch_id        閫昏緫閫氶亾ID
  * @param[in] src          鐘舵€佹簮鎿嶄綔鎺ュ彛
  * @param[in] hooks        浜嬩欢鍥炶皟閽╁瓙
  * @param[in] now_us       鏃堕棿鎴宠幏鍙栧嚱鏁?
  * @param[in] time_user    鏃堕棿鍥炶皟鐨勭敤鎴蜂笂涓嬫枃
  * @param[in] priority     浠茶浼樺厛绾?
  * @retval                 none
  */
void gimbal_channel_init(gimbal_channel_t *gc,
                         ch_uproto_bind_t *bind,
                         channel_manager_t *mgr,
                         uint8_t ch_id,
                         const gimbal_source_ops_t *src,
                         const gimbal_hooks_t *hooks,
                         uint64_t (*now_us)(void *user),
                         void *time_user,
                         uint8_t priority) {
    if(!gc || !bind || !mgr || !src)
        return;
    memset(gc, 0, sizeof(*gc));
    gc->bind = bind;
    gc->mgr = mgr;
    gc->ch_id = ch_id;
    gc->src = *src;
    if(hooks)
        gc->hooks = *hooks;
    gc->now_us = now_us;
    gc->time_user = time_user;
    channel_cfg_t cfg = {.mode = CH_MODE_PUSH, .reliable = 0, .expect_reply = 0, .priority = priority, .period_ms = 0};
    channel_hooks_t ch_hooks = {.on_rx = on_rx, .on_tick = on_tick, .on_cfg_change = NULL};
    channel_init(&gc->ch, gc->ch_id, &cfg, &ch_hooks, gc);
    channel_bind_transport(&gc->ch, gc->bind, gc->bind->msg_type);
    (void)chmgr_register(gc->mgr, &gc->ch);
}

/**
  * @brief                  浣跨敤閰嶇疆缁撴瀯浣撳垵濮嬪寲浜戝彴閫氶亾
  * @param[in] gc           浜戝彴閫氶亾瀹炰緥
  * @param[in] bind         uproto浼犺緭缁戝畾涓婁笅鏂?
  * @param[in] mgr          鎵€灞為€氶亾绠＄悊鍣?
  * @param[in] cfg          閫氶亾閰嶇疆鍙傛暟
  * @param[in] src          鐘舵€佹簮鎿嶄綔鎺ュ彛
  * @param[in] hooks        浜嬩欢鍥炶皟閽╁瓙
  * @param[in] now_us       鏃堕棿鎴宠幏鍙栧嚱鏁?
  * @param[in] time_user    鏃堕棿鍥炶皟鐨勭敤鎴蜂笂涓嬫枃
  * @retval                 none
  */
void gimbal_channel_init_ex(gimbal_channel_t *gc,
                            ch_uproto_bind_t *bind,
                            channel_manager_t *mgr,
                            const gimbal_channel_cfg_t *cfg,
                            const gimbal_source_ops_t *src,
                            const gimbal_hooks_t *hooks,
                            uint64_t (*now_us)(void *user),
                            void *time_user)
{
    if (!cfg) return;
    gimbal_source_ops_t s = src ? *src : (gimbal_source_ops_t){0};
    if (cfg->period_ms) s.period_ms = cfg->period_ms;
    gimbal_channel_init(gc, bind, mgr, cfg->ch_id, &s, hooks, now_us, time_user, cfg->priority);
}

/**
  * @brief                  绔嬪嵆鍙戝竷浜戝彴鐘舵€佸揩鐓?
  * @param[in] gc           浜戝彴閫氶亾瀹炰緥
  * @param[in] out          寰呭彂甯冪殑鐘舵€佹暟鎹紙NULL鏃朵娇鐢╯rc鑾峰彇锛?
  * @retval                 true=鎴愬姛锛宖alse=澶辫触
  */
bool gimbal_channel_publish(gimbal_channel_t *gc, const gimbal_state_t *opt) {
    if(!gc)
        return false;
    gimbal_state_t st;
    if(opt) {
        st = *opt;
    } else {
        if(!gc->src.get_state)
            return false;
        if(!gc->src.get_state(&st, gc->src.user))
            return false;
        if(st.ts_us == 0 && gc->now_us)
            st.ts_us = gc->now_us(gc->time_user);
    }
    uint8_t buf[128];  // 缓冲区，用于存储打包后的云台状态数据
    uint16_t len = gimbal_pack_state(buf, GIMBAL_SID_STATE,
                                     st.enc_yaw, st.enc_pitch,
                                     st.yaw_udeg, st.pitch_udeg, st.roll_udeg,
                                     st.ts_us);
    return ch_uproto_queue_notify(gc->bind, gc->ch_id, GIMBAL_SID_STATE, 0, buf, len);
}

/**
  * @brief                  Publish TFmini range data
  * @param[in] gc           gimbal channel instance
  * @param[in] data         TFmini range data
  * @retval                 true=ok, false=failed
  */
/**
  * @brief                  发布TFmini测距数据
  * @param[in] gc           云台通道实例
  * @param[in] data         TFmini测距数据
  * @retval                 true=成功，false=失败
  */
bool gimbal_channel_publish_tfmini(gimbal_channel_t *gc, const gimbal_tfmini_t *data) {
    if(!gc || !data)
        return false;
    uint8_t buf[2 + 2 + 2 + 2 + 2 + 8];
    comm_write_u16_le(&buf[0], GIMBAL_SID_TFMINI);
    comm_write_u16_le(&buf[2], data->distance_cm);
    comm_write_u16_le(&buf[4], data->strength);
    comm_write_u16_le(&buf[6], (uint16_t)data->temp_cdeg);
    comm_write_u16_le(&buf[8], data->status);
    comm_write_u64_le(&buf[10], data->ts_us);
    return ch_uproto_queue_notify(gc->bind, gc->ch_id, GIMBAL_SID_TFMINI, 0, buf, sizeof(buf));
}

/**
  * @brief                  发布裁判系统数据
  * @param[in] gc           云台通道实例
  * @param[in] game_progress        游戏进度
  * @param[in] stage_remain_time    阶段剩余时间
  * @param[in] game_result          比赛结果
  * @param[in] current_HP           当前血量
  * @param[in] shooter_42mm_heat    42mm发射热量
  * @param[in] ts_us                时间戳（微秒）
  * @retval                 true=成功，false=失败
  */
bool gimbal_channel_publish_referee(gimbal_channel_t *gc,
                                     uint8_t game_progress,
                                     uint16_t stage_remain_time,
                                     uint8_t game_result,
                                     uint16_t current_HP,
                                     uint16_t shooter_42mm_heat,
                                     uint64_t ts_us) {
    if(!gc)
        return false;
    uint8_t buf[2 + 1 + 2 + 1 + 2 + 2 + 8];  // SID + 裁判数据 + 时间戳
    uint16_t pos = 0;
    
    comm_write_u16_le(&buf[pos], GIMBAL_SID_REFEREE); pos += 2;
    buf[pos++] = game_progress;
    comm_write_u16_le(&buf[pos], stage_remain_time); pos += 2;
    buf[pos++] = game_result;
    comm_write_u16_le(&buf[pos], current_HP); pos += 2;
    comm_write_u16_le(&buf[pos], shooter_42mm_heat); pos += 2;
    comm_write_u64_le(&buf[pos], ts_us); pos += 8;
    
    return ch_uproto_queue_notify(gc->bind, gc->ch_id, GIMBAL_SID_REFEREE, 0, buf, pos);
}

static volatile gimbal_cmd_t g_latest = {0};
static volatile gimbal_big_yaw_cmd_t g_big_yaw_latest = {0};

/**
  * @brief                    璁剧疆鏈€鏂扮殑浜戝彴鎺у埗鍛戒护锛堟潵鑷富鏈猴級
  * @param[in] cmd            浜戝彴鎺у埗鍛戒护鎸囬拡
  * @retval                   none
  * @note                     绾跨▼瀹夊叏锛岄€傜敤浜嶤ortex-M鍗曞啓澶氳鍦烘櫙
  */
void gimbal_mailbox_set(const gimbal_cmd_t *cmd) {
    if(!cmd)
        return;
    g_latest.delta_yaw_udeg = cmd->delta_yaw_udeg;
    g_latest.delta_pitch_udeg = cmd->delta_pitch_udeg;
    g_latest.status = cmd->status;
    g_latest.ts_us = cmd->ts_us;
    g_latest.vx = cmd->vx;
    g_latest.vy = cmd->vy;
    g_latest.wz = cmd->wz;
    g_latest.flag_wz = cmd->flag_wz;
    g_latest.version++;
}

/**
  * @brief                    璇诲彇鏈€鏂扮殑浜戝彴鎺у埗鍛戒护
  * @param[out] out           杈撳嚭鍛戒护缂撳啿鍖?
  * @param[in,out] io_version 杈撳叆褰撳墠鐗堟湰鍙凤紝杈撳嚭鏂板懡浠ょ増鏈彿
  * @retval                   true=鏈夋柊鍛戒护锛宖alse=鏃犳柊鍛戒护鎴栧弬鏁版棤鏁?
  */
bool gimbal_mailbox_get(gimbal_cmd_t *out, uint32_t *io_version) {
    if(!out || !io_version)
        return false;
    uint32_t v = g_latest.version;
    if(v == *io_version)
        return false;
    /* copy fields */
    out->delta_yaw_udeg = g_latest.delta_yaw_udeg;
    out->delta_pitch_udeg = g_latest.delta_pitch_udeg;
    out->status = g_latest.status;
    out->ts_us = g_latest.ts_us;
    out->vx = g_latest.vx;
    out->vy = g_latest.vy;
    out->wz = g_latest.wz;
    out->flag_wz = g_latest.flag_wz;
    out->version = v;
    *io_version = v;
    return true;
}

void gimbal_big_yaw_mailbox_set(const gimbal_big_yaw_cmd_t *cmd) {
    if(!cmd)
        return;
    g_big_yaw_latest.delta_yaw_udeg = cmd->delta_yaw_udeg;
    g_big_yaw_latest.status = cmd->status;
    g_big_yaw_latest.ts_us = cmd->ts_us;
    g_big_yaw_latest.version++;
}

bool gimbal_big_yaw_mailbox_get(gimbal_big_yaw_cmd_t *out, uint32_t *io_version) {
    if(!out || !io_version)
        return false;
    uint32_t v = g_big_yaw_latest.version;
    if(v == *io_version)
        return false;
    out->delta_yaw_udeg = g_big_yaw_latest.delta_yaw_udeg;
    out->status = g_big_yaw_latest.status;
    out->ts_us = g_big_yaw_latest.ts_us;
    out->version = v;
    *io_version = v;
    return true;
}
