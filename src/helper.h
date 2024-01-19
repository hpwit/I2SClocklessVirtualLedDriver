
#define HOW_LONG(name,func) \
                        {\
                        long __time1__=ESP.getCycleCount();\
                        func;\
                        long __time2__=ESP.getCycleCount()-__time1__;\ 
                        printf("The function *** %s *** took %.2f ms or %.2f fps\n",name,(float)__time2__/24000,(float) 240000000/__time2__);\
                        }