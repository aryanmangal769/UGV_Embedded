#define Sine 11
#define Cosine 9
#define RevPin 10

int dir;

int Sine_counter = 0;
int Cosine_counter =0;

int sine;
int cosine;

int current_rev_ind;
int previous_rev_ind=0;

int rev_count;
int count;

int current_sine = 0;
int previous_sine =0;

int current_cosine =0;
int previous_cosine =0;

void setup()
{

  pinMode(Sine,INPUT);
  pinMode(Cosine,INPUT);

  pinMode(RevPin,INPUT);

  Serial.begin(2000000);
}

void loop()
{
  current_sine = digitalRead(Sine);
  current_cosine = digitalRead(Cosine);
  current_rev_ind = digitalRead(RevPin);


  //-------------------Rev Counter-----------------------
  
  if(current_rev_ind != previous_rev_ind && current_rev_ind > previous_rev_ind)
  {
    rev_count ++;
    count=0;

    Serial.print("***********************************rev_count = "); 
    Serial.println(rev_count);
    Serial.println("------------------");
  }

  previous_rev_ind = current_rev_ind;

  //-------------------Pulse Counter-----------------------


  if(current_sine > current_cosine)
  {
    dir = 1; 
  }

  if(current_sine < current_cosine)
  {
    dir = -1; 
  }
  

  if(current_sine!= previous_sine && current_sine > previous_sine) 
  {
    count += dir;

    Serial.print("count = "); 
    Serial.println(count);
    Serial.println("=====================");
  }
  previous_sine = current_sine;



  

}
