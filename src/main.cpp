/**************************************************************************
 * WiFi based location system to find home.
 * 
 * A Indoor Positioning System based on WiFi RSSI data to return to the 
 * right room of a hotel corridor. Maybe useless, but it works just fine!
 * 
 * Hague Nusseck @ electricidea
 * v1.1 05.June.2020
 * https://github.com/electricidea/M5Stack-Hotel-room-finder
 * 
 * Changelog:
 * v1.1 = - first published version
 * 
 * Distributed as-is; no warranty is given.
 * 
 ***************************************************************************/
#include <Arduino.h>

#include <M5Stack.h>
// install the library:
// pio lib install "M5Stack"

#include "WiFi.h"

// Free Fonts for nice looking fonts on the screen
#include "Free_Fonts.h"

// library for liniear and nonlinear fits
#include "curve_fit.h"

// maximum number of access points = maximum number of fits
const int max_fits = 40;
// array of of a number of fits 
curve_fit fits[max_fits] = curve_fit();

// position on the floor
double min_pos = 99999;
double max_pos = -99999;
// array for the calculated x positions along the floor
double *newx_array;
int n_newx = 0;

// the inverse intensity lookup table Map
double *IILTM;
int n_usable_APs = 0;

// the BSSID lookup table
typedef char cstring[100];  
cstring *BSSIDLT;
// the array for the square sums
double *square_sum_array;

// state machine index to switch between the menu states
int menu_state = 0;

#define STATE_START 1
#define STATE_MEASURE 2
#define STATE_GET_DATA 3
#define STATE_DATA 4
#define STATE_RUN 5

// value for the measurement along the floor
int measure_position = 0;

//==============================================================
// function forward declaration
uint16_t RGB2Color(uint8_t r, uint8_t g, uint8_t b);
void writeFile(fs::FS &fs, const char * path, const char * message);
void Clear_Screen();
void print_menu(int menu_index);
String split(String source, char delimiter, int location);
uint8_t collect_WiFi_data(String filename, bool append = true);
bool load_measurement(String filename);
bool load_floor_data();
String calculate_position();
bool analyze_measurements();


void setup() {
    // initialize the M5Stack object
    M5.begin();
    // configure the Lcd display
    M5.Lcd.setBrightness(100); //Brightness (0: Off - 255: Full)
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setTextSize(1);
    Clear_Screen();
    // configure centered String output
    M5.Lcd.setTextDatum(CC_DATUM);
    M5.Lcd.setFreeFont(FF2);
    M5.Lcd.drawString("Hotel room Finder", (int)(M5.Lcd.width()/2), (int)(M5.Lcd.height()/2), 1);
    M5.Lcd.setFreeFont(FF1);
    M5.Lcd.setTextColor(TFT_RED);
    M5.Lcd.drawString("Version 1.1 | 05.06.2020", (int)(M5.Lcd.width()/2), (int)(M5.Lcd.height()/2)+50, 1);
    // configure Top-Left oriented String output
    M5.Lcd.setTextDatum(TL_DATUM);
    M5.Lcd.setTextColor(TFT_WHITE);
    // Start Menu
    menu_state = 1;
    print_menu(menu_state);
}

void loop() {
  M5.update();

  // left Button
  if (M5.BtnA.wasPressed()){
    switch (menu_state) {
        case STATE_START: {   //  START -> MEASURE 
            Clear_Screen();
            menu_state = STATE_MEASURE;
            print_menu(menu_state);
            break;       
        }
        case STATE_MEASURE: {   //  MEASURE -> ADD
            Clear_Screen();
            M5.Lcd.println("Stand in front of the door\nand face the door.\n");
            M5.Lcd.println("got to the LEFT and press (<)");
            M5.Lcd.println("or to the RIGHT and press (>)");
            measure_position = 0;
            menu_state = STATE_GET_DATA;
            print_menu(menu_state);
            break;       
        }
        case STATE_GET_DATA: {   //  NEW -> < (-)
            Clear_Screen();
            ++measure_position;
            M5.Lcd.printf("measure %i steps away\n", measure_position);
            int n_WiFi_networks = collect_WiFi_data("/WiFi_data.txt");
            if(n_WiFi_networks > 0)
                M5.Lcd.printf("[OK] %i Networks found\n", n_WiFi_networks);
            else
                M5.Lcd.println("[ERR] unable to scan WiFi");
            print_menu(menu_state);
            break;       
        }
        case STATE_DATA: {   //  DATA -> DELETE
            Clear_Screen();
            M5.Lcd.println("Delete all measured data...");
            if(SD.remove("/WiFi_data.txt"))
              M5.Lcd.println("[OK] data deleted");
            else
              M5.Lcd.println("\n\n[ERR] unable to deleted data");
            writeFile(SD, "/WiFi_data.txt","pos;n;name;id;RSSI");
            measure_position = 0;
            n_usable_APs = 0;
            n_newx = 0;
            min_pos = 99999;
            max_pos = -99999;
            print_menu(menu_state);
            break;       
        }
        case STATE_RUN: {   //  RUN -> CHECK
            // check my position
            Clear_Screen();
            M5.Lcd.setTextDatum(CC_DATUM);
            M5.Lcd.setFreeFont(FF3); 
            M5.Lcd.drawString("Let me check...", (int)(M5.Lcd.width()/2), (int)(M5.Lcd.height()/2), 1);
            String pos_result = calculate_position();
            Clear_Screen();
            M5.Lcd.setTextDatum(CC_DATUM);
            M5.Lcd.setFreeFont(FF4); 
            M5.Lcd.drawString(pos_result.c_str(), (int)(M5.Lcd.width()/2), (int)(M5.Lcd.height()/2), 1);
            print_menu(menu_state);
            break;       
        }
    } 
  }


  // center Button
  if (M5.BtnB.wasPressed()){
    switch (menu_state) {
        case STATE_START: {   //  START -> RUN 
            Clear_Screen();
            M5.Lcd.setTextDatum(TL_DATUM);
            M5.Lcd.setFreeFont(FF1);
            // load floor data from SD card
            if(load_floor_data()) {
                Clear_Screen();
                M5.Lcd.println("\n\n      OK, ready to run");
                menu_state = STATE_RUN;
            } else
                M5.Lcd.println("Failed to load data");   
            print_menu(menu_state);
            break;       
        }
        case STATE_MEASURE: {   //  MEASURE -> BACK
            Clear_Screen();
            menu_state = STATE_START;
            print_menu(menu_state);
            break;       
        }
        case STATE_GET_DATA: {   //  MEASURE -> DONE
            Clear_Screen();
            // analyze measured data
            M5.Lcd.println("let's analyze the data");
            if(analyze_measurements())
                M5.Lcd.println("\n\n         OK, Success!");
            else
                M5.Lcd.println("\nSorry\nSomething went wrong.. :-(");
            menu_state = STATE_START;
            print_menu(menu_state);
            break;       
        }
        case STATE_DATA: {   //  DATA -> BACK
            Clear_Screen();
            menu_state = STATE_START;
            print_menu(menu_state);
            break;       
        }
        case STATE_RUN: {   //  RUN -> DONE
            Clear_Screen();
            menu_state = STATE_START;
            print_menu(menu_state);
            break;       
        }
    } 
  }

  
  // right Button
  if (M5.BtnC.wasPressed()){
    switch (menu_state) {
        case 1: {   //  START -> DATA 
            menu_state = STATE_DATA;
            print_menu(menu_state);
            break;       
        }
        case STATE_MEASURE: {   //  MEASURE -> NEW
            Clear_Screen();
            M5.Lcd.println("Delete all measured data...");
            if(!SD.remove("/WiFi_data.txt"))
              M5.Lcd.println("[ERR] unable to deleted data");
            writeFile(SD, "/WiFi_data.txt","pos;n;name;id;RSSI");
            M5.Lcd.println("\nReady for new measurements");
            M5.Lcd.println("\nStand in front of the door\nand face the door.\n");
            M5.Lcd.println("got to the LEFT and press (<)");
            M5.Lcd.println("or to the RIGHT and press (>)");
            measure_position = 0;
            menu_state = STATE_GET_DATA;
            print_menu(menu_state);
            break;       
        }
        case STATE_GET_DATA: {   //  NEW ->  > (+)
            Clear_Screen();
            --measure_position;
            M5.Lcd.printf("measure %i steps away\n", measure_position);
            int n_WiFi_networks = collect_WiFi_data("/WiFi_data.txt");
            if(n_WiFi_networks > 0)
              M5.Lcd.printf("[OK] %i Networks found\n", n_WiFi_networks);
            else
              M5.Lcd.println("\n\n[ERR] unable to scan WiFi");
            print_menu(menu_state);
            break;       
        }
        case STATE_DATA: {   //  DATA -> INFO
            Clear_Screen();
            M5.Lcd.printf("Data Info:\n\n");
            M5.Lcd.printf("usable APs: %i\n", n_usable_APs);
            M5.Lcd.printf("min x pos: %.1f\n", min_pos);
            M5.Lcd.printf("max x pos: %.1f\n", max_pos);
            print_menu(menu_state);
            break;       
        }
        case STATE_RUN: {   //  RUN -> Nothing
            break;       
        }
    } 
  }

  delay(50);
}

//==============================================================
// convert a RGB color into a LCD color value
uint16_t RGB2Color(uint8_t r, uint8_t g, uint8_t b) {
  return ((r / 8) << 11) | ((g / 4) << 5) | (b / 8);
}

//==============================================================
// split() returns a substring out of a source string
// defined by a delimiter and a location
// inspired by:
// http://wp.scalesoft.de/arduino-split/
String split(String source, char delimiter, int location) {
  String result = "";
  int locationCount = 0;
  int FromIndex = 0, ToIndex = -1;
  while (location >= locationCount) {
    FromIndex = ToIndex + 1;
    ToIndex = source.indexOf(delimiter,FromIndex);
    // if there is no delimiter anymore within the source string
    if(ToIndex == -1){
      // return the end of the string starting from the last delimiter
      // if the actual location is the target location
      if (location == locationCount) {
        result = source.substring(FromIndex);
      } else {
        // if not, return a empty string
        return "";
      }
    } else {
      // if it is not the last delimiter, but the actual
      // loacation is the target location, return the sub String
      if (location == locationCount)
        result = source.substring(FromIndex,ToIndex);
    }
    ++locationCount;
  }
  return result;
}

//==============================================================
// Scan for WiFi networks and save the SSID, BSSID and RSSI
// into a file on the SD card
// fix file name: "pos_data.txt"
uint8_t collect_WiFi_data(String filename, bool append){
    File file;
    if(append)
      file = SD.open(filename.c_str(), FILE_APPEND);
    else
      file = SD.open(filename.c_str(), FILE_WRITE);
    int n = WiFi.scanNetworks();
    if (n == 0) {
        M5.Lcd.println("[ERR] no networks found");
    } else {
        for (int i = 0; i < n; ++i) {
            // Print SSID, BSSID and RSSI for each network found
            file.printf("%i;%i;%s;%s;%i\n",measure_position , i+1, WiFi.SSID(i).c_str(), WiFi.BSSIDstr(i).c_str(), WiFi.RSSI(i));
            delay(10);
        }
    }
    file.close();
    return n;
}

//==============================================================
// Write Text into a file
void writeFile(fs::FS &fs, const char * path, const char * message){
    File file = fs.open(path, FILE_APPEND);
    if(!file){
        M5.Lcd.println("\n\n[ERR] Failed to open file");
        return;
    } else {
        if(!file.println(message))
            M5.Lcd.println("\n\n[ERR] Write failed");
        file.close();
    }
} 


//==============================================================
// Print a small menu at the bottom of the display above the buttons
void print_menu(int menu_index){
    M5.Lcd.fillRect(0, M5.Lcd.height()-25, M5.Lcd.width(), 25, RGB2Color(50,50,50));
    M5.Lcd.setCursor(0, 230);    
    M5.Lcd.setFreeFont(FF1);
    M5.Lcd.setTextColor(TFT_WHITE);
    switch (menu_index) {
      case 0: { // never used 
        M5.Lcd.print("      -       -        - ");
        break;       
      }
      case STATE_START: { // start menu
        M5.Lcd.print("   MEASURE   RUN     DATA"); 
        break;
      }
      case STATE_MEASURE: { // Submenu 1 for new Data collection  
        M5.Lcd.print("     ADD    BACK      NEW ");
        break;
      }
      case STATE_GET_DATA: { // Submenu 2 for new Data collection 
        M5.Lcd.print("      <     DONE       > ");
        break;
      }
      case STATE_DATA: { // DATA Submenu 
        M5.Lcd.print("   DELETE    BACK     INFO"); 
        break;
      }
      case STATE_RUN: { // RUN Submenu 
        M5.Lcd.print("    CHECK   DONE         "); 
        break;
      }
      default: { // should never been called
        M5.Lcd.print("      -       -        - ");
        break;
      }
    }
}


//==============================================================
// Clear the entire screen and add one row
// The added row is important. Otherwise the first row is not visible
void Clear_Screen(){
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("");
}


//==============================================================
// loads a stored measurement of positions and BSSID, RSSI data
// the data is used to learn the fits for each WiFi access point
// fix file name: "WiFi_data.txt"
bool load_measurement(String filename){
  // reset min and max for new measurements
  min_pos = 99999;
  max_pos = -99999;
  // reset all fits
  // and set the tag to -1 = not learned
  for(int i = 0; i < max_fits; ++i){
    // init as fith order polynomial
    fits[i].init(5);
    fits[i].reset();
    fits[i].tag = -1;
  }
  File file = SD.open("/WiFi_data.txt");
  if(!file){
      M5.Lcd.println("Failed to open file");
  } else {
      String line = "";
      while(file.available()){
        // file format:
        // pos;n;name;id;RSSI
        char chread = file.read();
        // ASCII printable characters (character code 32-127)
        if((chread >= 32) && (chread <= 127))
          line = line + String(chread);
        // CRLF = \r\n = ASCII code 13 and then ASCII code 10
        if((chread == '\r' || chread == '\n') && (line.length() > 0)){
          // get the BSSID at index 3
          String BSSID = split(line, ';', 3);
          // skip the header line
          if(BSSID != "id"){
            // get the position at index 0
            double pos = split(line, ';', 0).toDouble();
            if(pos > max_pos)
              max_pos = pos;
            if(pos < min_pos)
              min_pos = pos;
            // get the RSSI at index 4
            double RSSI = split(line, ';', 4).toDouble();
            int i = 0;
            bool OK_Flag = false;
            // check all fits if the BSSID exist
            // then let this fit learn the new data pair
            // otherwise, use the next unused fit for this BSSID
            do {
              if(fits[i].name == BSSID){
                fits[i].learn(pos, RSSI);
                OK_Flag = true;
              }
              if(fits[i].tag == -1){
                fits[i].tag = i;
                fits[i].name = BSSID;
                Serial.print(i);
                Serial.print(": ");
                Serial.println(BSSID);
                fits[i].learn(pos, RSSI);
                OK_Flag = true;
              }
              if(++i == max_fits){
                OK_Flag = true;
              }
            } while (!OK_Flag);
          }
          line = "";
        }
    }
    file.close();
    return true;
  }
  return false;
}


//==============================================================
// loads a stored floor data from SD card
// the data is used to find the room
// fix file name: "floor_data.txt"
bool load_floor_data(){
// load floor data from file
    M5.Lcd.printf("loading from file:\n  -->  /floor_data.txt\n");
    File file = SD.open("/floor_data.txt");
    if(!file){
      return false;
    } else {
      // File format:
      // n_newx
      // n_usable_APs
      // newx_array[0] ... newx_array[n_newx-1]
      // BSSIDLT[0] ... BSSIDLT[n_usable_APs-1]
      // IILTM[0] ... IILTM[((n_usable_APs-1)*n_newx)+(n_newx-1)]
      String line = "";
      int File_Block_index = 0;
      // 0 = header information with array dimensions
      // 1 = newx_array
      // 2 = BSSIDLT
      // 3 = IILTM
      int line_count = 0;
      while(file.available()){
          char chread = file.read();
          if(chread != '\n'){
            line = line + String(chread);
          } else {
            switch (File_Block_index) {
            // 0 = header information with array dimensions
            case 0:
              n_newx = split(line, ';', 0).toInt();
              n_usable_APs = split(line, ';', 1).toInt();     
              // free all daynamic arrays           
              free(newx_array);
              free(square_sum_array);
              free(BSSIDLT);
              free(IILTM);
              M5.Lcd.printf("new_x array size: %i \n", n_newx);
              M5.Lcd.printf("n_usable_APs: %i \n", n_usable_APs);
              Serial.printf("new_x array size: %i \n", n_newx);
              Serial.printf("n_usable_APs: %i \n", n_usable_APs);
              // allocate all dynamic arrays with the dimensions from the file
              newx_array = (double*) malloc(n_newx*sizeof(double));
              square_sum_array = (double*) malloc(n_newx*sizeof(double));
              BSSIDLT = (cstring*) malloc(n_usable_APs*sizeof(cstring));
              IILTM = (double*) malloc(n_newx*n_usable_APs*sizeof(double));
              // stop, if allocation fails
              if(!newx_array || !square_sum_array || !IILTM || !BSSIDLT){
                M5.Lcd.printf("[ERR] unable to allocate memory\n");
                delay(5000);
                // stop processing readed data from file
                File_Block_index = -1;
                file.close();
                return false;
              } else{
                Serial.println("done: header!");
                Serial.println("read newx_array data...");
                File_Block_index = 1;
              }
              break;
            
            // 1 = newx_array data
            case 1:
              if(line != ""){
                // read the new-x values line by line
                newx_array[line_count++] = split(line, ';', 0).toDouble();
                if(line_count == n_newx){
                  Serial.println("done: read newx_array!");
                  Serial.println("read BSSIDLT data...");
                  File_Block_index = 2;
                  line_count = 0;
                }
              }
              break;
              
            // 2 = BSSIDLT data
            case 2:
              if(line != ""){
                // read the BSSID values line by line
                strcpy(BSSIDLT[line_count++], split(line, ';', 0).c_str());
                if(line_count == n_usable_APs){
                  Serial.println("done: read BSSIDLT!");
                  Serial.println("read IILTM data...");
                  File_Block_index = 3;
                  line_count = 0;
                }
              }
              break;
              
            // 3 = IILTM data
            case 3:
              if(line != ""){
                // read the IILTM data
                for(int i=0; i < n_usable_APs; ++i){
                  IILTM[(i*n_newx)+line_count] = split(line, ';', i).toDouble();
                }
                ++line_count;
                if(line_count == n_newx){
                  Serial.println("done: read IILTM!");
                  File_Block_index = -1;
                  line_count = 0;
                }
              }
              break;
            
            default:
              break;
            }
            line = "";
          }
      }
      file.close();
    } 
    return true; 
}

//==============================================================
// scan fout times the available APS
// Calculate the best fitting positon based on the IILTM
// Return the number as text, or text if the position can't be calculated
String calculate_position(){
  String result = "...";
  // scan APs and save to File "pos_data.txt"
  // four times...
  int n_WiFi_networks = collect_WiFi_data("/pos_data.txt", false);
  collect_WiFi_data("/pos_data.txt");
  collect_WiFi_data("/pos_data.txt");
  collect_WiFi_data("/pos_data.txt");

  // open the file and go throw all APs...
  File file = SD.open("/pos_data.txt");
  if(!file || n_newx == 0 || n_usable_APs == 0 || n_WiFi_networks == 0){
    // without a file or without any APs, we are unable to find the room
    return "No idea :-(";
  } else {
    // Because we scanned four times, we have to average the RSSI data
    // This can be done with the fit-class ()
    // reset all fits
    // and set the tag to -1 = not learned
    for(int i = 0; i < max_fits; ++i){
      fits[i].init(0);
      fits[i].reset();
      fits[i].tag = -1;
    }
    String line = "";
    while(file.available()){
      char chread = file.read();
      // ASCII printable characters (character code 32-127)
      if((chread >= 32) && (chread <= 127))
        line = line + String(chread);
      // CRLF = \r\n = ASCII code 13 and then ASCII code 10
      if((chread == '\r' || chread == '\n') && (line.length() > 0)){
        // file format:
        // pos;n;name;BSSID;RSSI
        String BSSID = split(line, ';', 3);
        // skip the header line
        if(BSSID != "id"){
          double RSSI = split(line, ';', 4).toDouble();
          // find AP in BSSIDLT
          int AP_index = -1;
          for(int i = 0; i < n_usable_APs; ++i){
            if(strcmp(BSSIDLT[i], BSSID.c_str()) == 0)
              AP_index = i;
          }
          if(AP_index > -1){
            fits[AP_index].learn(0.0, RSSI);
            fits[AP_index].tag = AP_index;
          }
        }
        line = "";
      }
    }
    file.close();
    // Now, the fits are filled with the average RSSI data from the APs
    // Time to calculate the square sum array:
    for(int x = 0; x < n_newx; ++x)
      square_sum_array[x] = 0.0;
    for(int AP_index = 0; AP_index < n_usable_APs; ++AP_index){
      if(fits[AP_index].tag > -1){
        for(int x = 0; x < n_newx; ++x){
          // predict(0.0) will return the average value because 
          // the fit is initialized with degree = 0
          double RSSI_diff = (fits[AP_index].predict(0.0) - IILTM[(AP_index*n_newx)+x]);
          square_sum_array[x] += (RSSI_diff*RSSI_diff);
        }
      }
    }
    // Finally find the minimum of the square sums:
    double best_pos = newx_array[0];
    double min_sum = square_sum_array[0];
    for(int x = 0; x < n_newx; ++x){
      if(square_sum_array[x] < min_sum){
        min_sum = square_sum_array[x];
        best_pos = newx_array[x];
      }
    }
    // If best pos is the first or the last position of the new x array
    // then we can say that we are far away, because we might don't know 
    // the right value of the distance
    if(best_pos == newx_array[0] || best_pos == newx_array[n_newx-1])
      result = "far away...";
    else
      result = String(best_pos).c_str();
  }
  return result;
}

//==============================================================
// load the measurements from SD card (file: /WiFi_data.txt)
// build the new-x array, the BSSIDLT and the IILTM
// return true if the procedure was succesfull
bool analyze_measurements(){
  bool result = false;
  M5.Lcd.printf("Reading file:\n --> /WiFi_data.txt\n");
  // load measured data from file and let the fits learn...
  if(load_measurement("/WiFi_data.txt")){
    M5.Lcd.printf("Analyze AP data\n");
    // build new_x array....
    // free the daynamic arrays
    free(newx_array);
    free(square_sum_array);
    // get the position range out of the data
    int x_range = round(max_pos - min_pos);
    n_newx = x_range *2;
    // allocate the memory for the array with the new size
    newx_array = (double*) malloc(n_newx*sizeof(double));
    square_sum_array = (double*) malloc(n_newx*sizeof(double));
    // if the memory allocation failed
    if(!newx_array || !square_sum_array)
      return false;
    else {
      // fill the newx_array with the fine position steps
      for(int i=0; i < n_newx; ++i){
        newx_array[i] = min_pos + (i*((double)x_range / (double)n_newx));
      }
      // check for usable APs out of the fits
      // criteria:
      // at least 6 valid data points
      //    --> fith order polynome should have at least 6 values
      // estimated min and max y values should not be out of bounds [-25 .. -95]
      // a minimum of 15dBm amplitude over the data range is required
      n_usable_APs = 0;
      for(int i = 0; i < max_fits; ++i){
        // check all fits for criteria
        double min_y = fits[i].estimate_min_y();
        double max_y = fits[i].estimate_max_y();
        if(fits[i].tag > -1){
          if((fits[i].count() < 6) ||               
              (min_y < -95.0) || (max_y > -25.0) ||  
              (fabs(max_y - min_y) < 15)) {          
                fits[i].reset();
                fits[i].name = "";
                fits[i].tag = -1;
          }
        }
        if(fits[i].tag > -1){
          Serial.printf("%i: N: %i min: %.2f max: %.2f\n", i, fits[i].count(), fits[i].estimate_min_y(), fits[i].estimate_max_y());
          ++n_usable_APs;
        }
      }

      // build Inverse Intensity Lookup Table Map (IILTM)
      // ....
      M5.Lcd.printf("Build IILTM and BSSIDLT\n");
      Serial.println("Build the IILTM and the BSSIDLT:");
      if(n_usable_APs > 0){
        Serial.printf("number of usable APs: %i \n", n_usable_APs);
        // free the daynamic arrays
        free(IILTM);
        free(BSSIDLT);
        // allocate the memory for the array with the new size
        BSSIDLT = (cstring*) malloc(n_usable_APs*sizeof(cstring));
        IILTM = (double*) malloc(n_newx*n_usable_APs*sizeof(double));
        // if the memory allocation failed
        if(!IILTM || !BSSIDLT) {
          Serial.println("[ERR] malloc failed");
          return false;
        } else {
          int AP_count = 0;
          for(int i = 0; i < max_fits; ++i){
            //Serial.printf("check: %i: \n", i);
            if(fits[i].tag > -1){
              //Serial.printf("tag > -1 --> %i: %s\n", AP_count, fits[i].name.c_str());
              strcpy(BSSIDLT[AP_count], fits[i].name.c_str());
              for(int x = 0; x < n_newx; ++x){
                // -95dBm for x values outside the learned range
                IILTM[(AP_count*n_newx)+x] = fits[i].predict(newx_array[x], -95.0);
              }
              ++AP_count;
            }
          }
        }
      } else {
        M5.Lcd.printf("no usable APs found!\n");
        Serial.println("no usable APs found!");
        return false;
      }

      Serial.println("the BSSIDLT:");
      for(int i = 0; i < n_usable_APs; ++i){
        Serial.printf("%i: %s\n", i, BSSIDLT[i]);
      }

      Serial.println("the IILTM:");
      for(int x = 0; x < n_newx; ++x){
        Serial.printf("\n%.2f", newx_array[x]);
        for(int i = 0; i < n_usable_APs; ++i){
          Serial.printf(" %.2f", IILTM[(i*n_newx)+x]);
        }
      }
      // save floor data to file
      M5.Lcd.printf("Writing to file:\n --> /floor_data.txt\n");
      File file = SD.open("/floor_data.txt", FILE_WRITE);
      if(!file){
          M5.Lcd.println("Failed to open file");
          return false;
      } else {
        // File format:
        // n_newx
        // n_usable_APs
        // newx_array[0] ... newx_array[n_newx-1]
        // BSSIDLT[0] ... BSSIDLT[n_usable_APs-1]
        // IILTM[0] ... IILTM[((n_usable_APs-1)*n_newx)+(n_newx-1)]
        // header with dimensions
        file.printf("%i;%i\n",n_newx, n_usable_APs);
        // save newx_array
        for(int x=0; x < n_newx; ++x){
          file.printf("%.6f\n",newx_array[x]);
        }
        // save BSSIDLT array
        for(int i=0; i < n_usable_APs; ++i){
          file.printf("%s\n",BSSIDLT[i]);
        }
        // save IILTM array
        for(int x=0; x < n_newx; ++x){
          file.printf("\n%.6f",IILTM[(0*n_newx)+x]);
        for(int i=1; i < n_usable_APs; ++i){
            file.printf(";%.6f",IILTM[(i*n_newx)+x]);
          }
        }
        file.printf("\n");
        file.close();
      }

      M5.Lcd.println("done..");
      Serial.println("");
      Serial.println("done!");
    }
    result = true;
  }
  return result;
}
