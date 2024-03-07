

struct bmsTelegrams {
  unsigned short Id;
  unsigned char len;
  unsigned long timestamp;
  short tdata[8];
};

struct bmsCommunication {
  unsigned long timestamp;
  unsigned long actual_latency;
  unsigned long maxdelay;
};

struct cella {
  bool  first_value_received;
  unsigned short v;
  float v_filtered;
  float max_deviation_from_avg;
  unsigned short vmin;
  unsigned short vmax;
  float current_at_vmax;
  float current_at_vmin;
};



