#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <stdio.h>

int updateSensors(char *host)
{
	struct addrinfo *addrinfo;
	int ret;
	int fd;
	FILE *fp;

	if((ret = getaddrinfo(host, "http", NULL, &addrinfo)) != 0) {
		fprintf(stderr, "getaddrinfo(): %s\n", gai_strerror(ret));
		return -1;
	}

	if((fd = socket(addrinfo->ai_family, addrinfo->ai_socktype,
			addrinfo->ai_protocol)) < 0) {
		perror("socket()");
		freeaddrinfo(addrinfo);
		return -1;
	}

	if(connect(fd, addrinfo->ai_addr, addrinfo->ai_addrlen) < 0) {
		perror("connect()");
		close(fd);
		freeaddrinfo(addrinfo);
		return -1;
	}

	if((fp = fdopen(fd, "r+")) == NULL) {
		perror("connect()");
		close(fd);
		freeaddrinfo(addrinfo);
		return -1;
	}

	fprintf(fp, "GET / HTTP/1.0\r\n"
		    "Host: %s\r\n"
		    "Connection: close\r\n"
		    "\r\n", host);
	while(!feof(fp)) {
		fputc(fgetc(fp), stdout);
	}
	fclose(fp);
	close(fd);
	freeaddrinfo(addrinfo);

	return 0;
}

int main()
{
	return updateSensors("www.google.com");
	//return updateSensors("localhost");
}
