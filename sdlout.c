#include <SDL2/SDL.h>
#include "raytrasdf.h"

static inline bool init_stuff(SDL_Window **win, SDL_Surface **sur, uint w, uint h){
    if(SDL_Init(SDL_INIT_VIDEO|SDL_INIT_EVENTS)!=0){
        fprintf(stderr,"SDL_Init Error: %s\n",SDL_GetError());
        return false;
    }
    *win=SDL_CreateWindow("raytrasdf",100,100,w,h,SDL_WINDOW_SHOWN);
    if(win==NULL){
        fprintf(stderr,"SDL_CreateWindow Error: %s\n",SDL_GetError());
        return false;
    }
    *sur=SDL_GetWindowSurface(*win);
    if(sur==NULL){
        fprintf(stderr,"SDL_GetWindowSurface Error: %s\n",SDL_GetError());
        SDL_DestroyWindow(*win);
        SDL_Quit();
        return false;
    }
    return true;
}

void *draw_stuff(struct draw_args *args){
    struct bgra{
        uint8_t b,g,r,a;
    };
    SDL_Event event;
    SDL_Window *win;
    SDL_Surface *sur;
    if(init_stuff(&win,&sur,args->w,args->h)){
        struct bgra *pixels=(struct bgra*)sur->pixels;
        while(1){
            while(SDL_PollEvent(&event))
                if(event.type==SDL_QUIT){
                    SDL_DestroyWindow(win);
                    SDL_Quit();
                    return NULL;
                }
            for(uint i=0;i<(args->w*args->h);++i){
                pixels[i].r=args->pic[i].r;
                pixels[i].g=args->pic[i].g;
                pixels[i].b=args->pic[i].b;
                pixels[i].a=0;
            }
            SDL_UpdateWindowSurface(win);
            SDL_Delay(200);
        }
    }
    return NULL;
}
