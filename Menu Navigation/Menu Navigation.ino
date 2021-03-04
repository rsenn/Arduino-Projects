
// Keep code clean 

//Importing all rquired Libraries
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

//Pin description
#define upbtn  10
#define selectbtn  11
#define downbtn  12
#define backlight 9

 
//All variables here
boolean backlight_state = true;  //Stores the backlight state 
int contrast = 50;               //Stores the Contrast value (Default)
int selectbtn_state = 1;
int upbtn_state = 1;
int downbtn_state = 1;
int menuitem = 1;               //It is the item in the menu the cursor is currently on
int page = 1;                   //
boolean selected = false;       //Shows selsection state
int last_upbtn_state = 1;
int last_selectbtn_state = 1;
int last_downbtn_state = 1;

//Initialise the lcd library 
Adafruit_PCD8544 display = Adafruit_PCD8544( 7, 6, 5, 4, 3);

/*
Function List :
bklight_on
bklight_off
drawMenu
reset
pageup
pagedown
*/

void setup() {
  //Debug codes uncomment in dev builds
  Serial.begin(9600); //Serial Debug
  pinMode(13, OUTPUT); //LED

  //Pin configurations
  pinMode(selectbtn, INPUT_PULLUP); //When button pressed pin goes low
  pinMode(upbtn, INPUT_PULLUP);
  pinMode(downbtn, INPUT_PULLUP);
  pinMode(backlight, OUTPUT);

  //Display initialization code
  display.begin();      
  display.setContrast(contrast);  //Sets contrast to 50
  display.clearDisplay();         //Clears display
  display.display();  

}

void loop() {
  //Clearing all pressed pins
  selectbtn_state = 1;
  upbtn_state = 1;
  downbtn_state = 1;
  
  // This code draws a menu then checks if any button is pressed ,if any gets pressed changes the variables and draws the new menu 
  drawMenu();
  
  //Single button press checker
  pressChecker();

  //Gives directions to the current button pressed
  if(selectbtn_state==0){if (selected==false){selected=true;}else{selected=false;}}
  
  if(upbtn_state==0 && selected == false)
    { //If up button is pressed but no option selected
        if (page-1 >0){page--;} //decrements the page
        else {page=3;}//puts page to last option 
    }
  else if(downbtn_state==0 && selected == false)
    { //If down button is pressed but no option selected
        if (page==3){page=1;} //increment the page
        else {page++;}//puts page to first option 
    }
  else if(upbtn_state==0 && selected == true)
    { //If up button is pressed but option selected
        pageup();
    } 
  else if(downbtn_state==0 && selected == true)
    { //If down button is pressed but option selected
        pagedown();
    } 
  
  delay(20); //To deal with debounce
  int r=(selected==true)?1:0;
  digitalWrite(13,r);
}

//Backlight ON and OFF functions
void bklight_on(){digitalWrite(backlight,LOW);backlight_state = true;}
void bklight_off(){digitalWrite(backlight,LOW);backlight_state = false;}
void bklight_toggle(){digitalWrite(backlight,!digitalRead(backlight));if (backlight_state==false){backlight_state=true;}else{backlight_state=false;}}

//Checks which button is pressed
void pressChecker(){
  while(selectbtn_state && upbtn_state && downbtn_state)
  {
    selectbtn_state = digitalRead(selectbtn);
    upbtn_state = digitalRead(upbtn);
    downbtn_state = digitalRead(downbtn);
  }
  
      //debounce code BIG ISSUE
      if (selectbtn_state != last_selectbtn_state) 
      {
        while(selectbtn_state!=digitalRead(selectbtn)){}
        delay(50);
      }
      last_selectbtn_state = selectbtn_state;

      if (upbtn_state != last_upbtn_state) 
      {
        while(upbtn_state!=digitalRead(upbtn)){}
        delay(50);
      }
      last_upbtn_state = upbtn_state;

      if (downbtn_state != last_downbtn_state) 
      {
        while(downbtn_state!=digitalRead(downbtn)){}
        delay(50);
      }
      last_downbtn_state = downbtn_state;
  
  //debug code
  if (selectbtn_state ==0){
    Serial.println("pressed select");
  }
  else if (upbtn_state ==0){
    Serial.println("pressed up");
  }
  else  if (downbtn_state ==0){
    Serial.println("pressed down");
  }
  delay(150);
}

//When up is pressed after selecting an option
void pageup(){
  if (page==1){
    contrast+=10;
    display.setContrast(contrast);  //Sets contrast to new contrast
    display.display();
  }
  else if (page==2){bklight_toggle();}
  else if (page==3){reset();}
}

//When down is pressed after selecting an option
void pagedown(){
  if (page==1){
    contrast-=10;
    display.setContrast(contrast);  //Sets contrast to new contrast
    display.display();
  }
  else if (page==2){bklight_toggle();}
  else if (page==3){reset();}
}

//Reset code
void reset(){
  contrast = 50;
  display.setContrast(contrast);
  bklight_on();
  display.display();
  Serial.println("reset");
}
  

//Draws the menu with current states
void drawMenu(){
  //This function draws the menu according to the current menu and page state
  display.setTextSize(0.2);
  display.clearDisplay();
  display.setTextColor(BLACK, WHITE);
  display.setCursor(25, 0);
  display.print("MENU");
  //display.drawFastHLine(0,10,83,BLACK);
  //display.setCursor(0, 15);
  
  for(int i=0;i<=3;i++){
    display.setCursor(0, 15 + 10*(i-1));
    //If selsected page comes print in another colour
    if (i==page){display.setTextColor(WHITE, BLACK);} else{display.setTextColor(BLACK, WHITE);}
    
    switch(i){
      case 1:display.print("Contrast");display.setTextColor(BLACK, WHITE); display.print(" ");display.print(contrast);break;
      case 2:display.print("Light"); display.setTextColor(BLACK, WHITE);display.print(" ");(backlight_state) ? display.print("ON") : display.print("OFF") ;break;
      case 3:display.print("Reset"); display.setTextColor(BLACK, WHITE);break;
    }
    
  }
  display.display();
}
