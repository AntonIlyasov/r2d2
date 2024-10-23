#define TOF_CMD_OFF         2
#define TOF_CMD_ON          1
#define TOF_CMD_SAVE_FHD    4
#define TOF_CMD_SAVE_DEPTH  8

/*Устанавливает бит в заданную позицию в INT 32*/
inline void set_bit(uint8_t &dst, bool val, uint8_t  bit) {
  val ? dst |= (1 << bit) : dst &= ~(1 << bit);
}

/*Возвращает значение бита, стоящего на данной позиции в данном байте*/
inline bool check_bit(uint8_t byte, uint8_t bit, bool debug) {
  byte &= (1 << bit);
  if(bit > 7) {
    std::cout << "WRONG BIT NUMBER " << bit << '\n';
    return false;
  }
  if (debug) {
    if (byte != 0 ) {
      printf ("1");
    } else {
      printf ("0");
    }
  }
  return byte;
}

void setAcceptedAnswer(uint8_t & answerByte, uint8_t cmd, bool accepted) {
  switch (cmd)
  {
  case TOF_CMD_ON:
    set_bit(answerByte, accepted, 0);
    break;
  case TOF_CMD_OFF:
    set_bit(answerByte, accepted, 4);
    break;
  case TOF_CMD_SAVE_FHD:
    set_bit(answerByte, accepted, 0);
    break;
  case TOF_CMD_SAVE_DEPTH:
    set_bit(answerByte, accepted, 4);
    break;
  
  default:
    break;
  }
}

void setDoneAnswer(uint8_t & answerByte, uint8_t cmd, bool done) {
  switch (cmd)
  {
  case TOF_CMD_ON:
    set_bit(answerByte, done, 1);
    break;
  case TOF_CMD_OFF:
    set_bit(answerByte, done, 5);
    break;
  case TOF_CMD_SAVE_FHD:
    set_bit(answerByte, done, 1);
    break;
  case TOF_CMD_SAVE_DEPTH:
    set_bit(answerByte, done, 5);
    break;
  
  default:
    break;
  }
}

void setErrAnswer(uint8_t & answerByte, uint8_t cmd) {
  switch (cmd)
  {
  case TOF_CMD_ON:
    set_bit(answerByte, true, 2);
    break;
  case TOF_CMD_OFF:
    set_bit(answerByte, true, 6);
    break;
  case TOF_CMD_SAVE_FHD:
    set_bit(answerByte, true, 2);
    break;
  case TOF_CMD_SAVE_DEPTH:
    set_bit(answerByte, true, 6);
    break;
  
  default:
    break;
  }
}