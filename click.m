#import <Foundation/Foundation.h>
#import <ApplicationServices/ApplicationServices.h>

int main(int argc, char *argv[]) {
    NSAutoreleasePool *pool = [[NSAutoreleasePool alloc] init];
    NSUserDefaults *args = [NSUserDefaults standardUserDefaults];

    int x = [args integerForKey:@"x"];
    int y = [args integerForKey:@"y"];

    if(x > 600){
        x = x - 655;
    }
    if(y > 600){
        y = y - 655;
    }

    CGEventRef ourEvent = CGEventCreate(NULL);
    CGPoint point = CGEventGetLocation(ourEvent);
    CFRelease(ourEvent);
    NSLog(@"%f %f", (float)point.x, (float)point.y);

    int mult = 2;

    point.x -= (x * mult);
    point.y += (y * mult);

    if(point.x < 0){
        point.x = 0;
    }

    if(point.x > 1279){
        point.x = 1279;
    }

    if(point.y < 0){
        point.y = 0;
    }

    if(point.y > 799){
        point.y = 799;
    }

    CGPostMouseEvent( point, 1, 1, 0 );

    [pool release];
    return 0;
}