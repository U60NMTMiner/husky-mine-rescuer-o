#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>

/*
Useful script for looking up addresses. I suppose you could also
cat /etc/hosts | grep $1 | awk '{print $1}',
but wheres the fun in that?
*/

int main(int argc, char **argv) {
    struct addrinfo *result;
    if (getaddrinfo(argv[1], NULL, NULL, &result)) {
        fprintf(stderr, "lookup error on %s\n", argv[1]);
        return 1;
    }

    struct sockaddr_in *addr = (struct sockaddr_in *)result->ai_addr;
    const unsigned char *ip_data = (const unsigned char *)&addr->sin_addr.s_addr;

    for (int i = 0; i < 4; i++)
        printf("%s%d", i == 0 ? "" : ".", ip_data[i]);
}
