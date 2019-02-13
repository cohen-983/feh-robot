#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>

int main(void)
{

    float x,y;

    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    while( true )
    {
        if( LCD.Touch(&x,&y) )
        {
            LCD.WriteLine("Hello, suckers!");
            Sleep( 100 );
        }
    }
    return 0;
}
