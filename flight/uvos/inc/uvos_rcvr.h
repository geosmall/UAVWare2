#ifndef UVOS_RCVR_H
#define UVOS_RCVR_H

struct uvos_rcvr_driver {
  void    ( *init )( uint32_t id );
  int32_t ( *read )( uint32_t id, uint8_t channel );
  // xSemaphoreHandle (*get_semaphore)(uint32_t id, uint8_t channel);
  uint32_t  ( *get_flag )( uint32_t id );
  uint8_t ( *get_quality )( uint32_t id );
};

/* Public Functions */
extern int32_t UVOS_RCVR_Read( uint32_t rcvr_id, uint8_t channel );
extern uint8_t UVOS_RCVR_GetQuality( uint32_t rcvr_id );
// extern xSemaphoreHandle UVOS_RCVR_GetSemaphore(uint32_t rcvr_id, uint8_t channel);
extern bool UVOS_RCVR_GetFrameAvailableFlag( uint32_t rcvr_id );

/*! Define error codes for UVOS_RCVR_Get */
enum UVOS_RCVR_errors {
  /*! Indicates that a failsafe condition or missing receiver detected for that channel */
  UVOS_RCVR_TIMEOUT  = -1,
  /*! Channel is invalid for this driver (usually out of range supported) */
  UVOS_RCVR_INVALID  = -2,
  /*! Indicates that the driver for this channel has not been initialized */
  UVOS_RCVR_NODRIVER = -3
};

// Enumeration options for field ChannelGroups
typedef enum {
  MANUALCONTROLSETTINGS_CHANNELGROUPS_PPM = 0,
  MANUALCONTROLSETTINGS_CHANNELGROUPS_SBUS = 1,
  MANUALCONTROLSETTINGS_CHANNELGROUPS_IBUS = 2,
  MANUALCONTROLSETTINGS_CHANNELGROUPS_NONE = 3
} ManualControlSettingsChannelGroupsOptions;

#endif /* UVOS_RCVR_H */

