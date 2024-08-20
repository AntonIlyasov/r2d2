#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "lib.h"

#include <ros/ros.h>
#include <iostream>

int main(int argc, char **argv) {
  try{
    std::cout << "\n\033[1;32m╔═══════════════════════════════════════╗\033[0m"
              << "\n\033[1;32m║          CAN_SEND is running!         ║\033[0m" 
              << "\n\033[1;32m╚═══════════════════════════════════════╝\033[0m\n";
    ros::init(argc, argv, "can_send");

    int s; /* can raw socket */
    int required_mtu;
    int mtu;
    int enable_canfx = 1;
    struct sockaddr_can addr;
    struct can_raw_vcid_options vcid_opts = {
      .flags = CAN_RAW_XL_VCID_TX_PASS,
    };
    static cu_t cu;
    struct ifreq ifr;

    std::string msgToSendViaCAN = "";
    ros::param::get("/_msg", msgToSendViaCAN);

    /* parse CAN frame */
    char *strParam = const_cast<char*>(msgToSendViaCAN.c_str());
    required_mtu = parse_canframe(strParam, &cu);
    if (!required_mtu) {
      fprintf(stderr, "\nWrong CAN-frame format!\n\n");
      return 1;
    }

    /* open socket */
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
      perror("socket");
      return 1;
    }

    strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
    if (!ifr.ifr_ifindex) {
      perror("if_nametoindex");
      return 1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (required_mtu > (int)CAN_MTU) {
      /* check if the frame fits into the CAN netdevice */
      if (ioctl(s, SIOCGIFMTU, &ifr) < 0) {
        perror("SIOCGIFMTU");
        return 1;
      }
      mtu = ifr.ifr_mtu;

      if (mtu == CANFD_MTU) {
        /* interface is ok - try to switch the socket into CAN FD mode */
        if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                &enable_canfx, sizeof(enable_canfx))){
          printf("error when enabling CAN FD support\n");
          return 1;
        }
      }

      if (mtu >= CANXL_MIN_MTU) {
        /* interface is ok - try to switch the socket into CAN XL mode */
        if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_XL_FRAMES,
                &enable_canfx, sizeof(enable_canfx))){
          printf("error when enabling CAN XL support\n");
          return 1;
        }
        /* try to enable the CAN XL VCID pass through mode */
        if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_XL_VCID_OPTS,
                &vcid_opts, sizeof(vcid_opts))) {
          printf("error when enabling CAN XL VCID pass through\n");
          return 1;
        }
      }
    }

    /* ensure discrete CAN FD length values 0..8, 12, 16, 20, 24, 32, 64 */
    if (required_mtu == CANFD_MTU)
      cu.fd.len = can_fd_dlc2len(can_fd_len2dlc(cu.fd.len));

    /* CAN XL frames need real frame length for sending */
    if (required_mtu == CANXL_MTU)
      required_mtu = CANXL_HDR_SIZE + cu.xl.len;

    /*
    * disable default receive filter on this RAW socket This is
    * obsolete as we do not read from the socket at all, but for
    * this reason we can remove the receive list in the Kernel to
    * save a little (really a very little!) CPU usage.
    */
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      perror("bind");
      return 1;
    }

    /* send frame */
    if (write(s, &cu, required_mtu) != required_mtu) {
      perror("write");
      return 1;
    }

    close(s);

    std::cout << "OK canopen_node\n";
  } catch (std::exception e){
    std::cerr << "\nExeption: " << e.what() << std::endl;
  }
  return 0;
}