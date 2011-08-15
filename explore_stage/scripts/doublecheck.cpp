#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    if(argc >= 3)
    {
        double a = atof(argv[1]);
        double b = atof(argv[2]);
        
        if(a > b)
            printf("greater");
        else if(a < b)
            printf("less");
        else
            printf("equal");
        
        return 0;
    }
    
    return 1;
}
