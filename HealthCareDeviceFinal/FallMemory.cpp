#include "FallMemory.h"
#include <Preferences.h>

static Preferences prefs;

void FallMemory::begin(){
  prefs.begin("fallmem", false);
  size = prefs.getUInt("size", 0);
  if(size > FALL_MEM_MAX) size = 0;

  for(uint16_t i=0;i<size;i++){
    char key[16];
    snprintf(key,sizeof(key),"p%u",i);
    prefs.getBytes(key, &patterns[i], sizeof(FallPattern));
  }
}

bool FallMemory::hasData(){
  return size > 0;
}

uint16_t FallMemory::count(){
  return size;
}

/* ===== similarity between 2 patterns ===== */
float FallMemory::similarity(const FallPattern& a,
                             const FallPattern& b)
{
  auto vecSim = [](float x1, float y1, float z1,
                   float x2, float y2, float z2)
  {
    float dot = x1*x2 + y1*y2 + z1*z2;
    float n1  = sqrt(x1*x1 + y1*y1 + z1*z1);
    float n2  = sqrt(x2*x2 + y2*y2 + z2*z2);

    if(n1 < 1e-3f || n2 < 1e-3f) return 0.0f;

    float c = dot / (n1 * n2);   // cosine similarity
    if(c < 0) c = 0;
    if(c > 1) c = 1;
    return c;
  };

  float simAccel = vecSim(
    a.ax, a.ay, a.az,
    b.ax, b.ay, b.az
  );

  float simGyro = vecSim(
    a.gx, a.gy, a.gz,
    b.gx, b.gy, b.gz
  );

  // Gia tốc quan trọng hơn gyro
  return 0.6f * simAccel + 0.4f * simGyro;
}


/* ===== check false fall ===== */
bool FallMemory::isFalseFall(const FallPattern& cur){
  for(uint16_t i=0;i<size;i++){
    if(similarity(cur, patterns[i]) >= FALL_SIM_THRESHOLD){
      return true;
    }
  }
  return false;
}

/* ===== add false fall ===== */
void FallMemory::addFalseFall(const FallPattern& cur){
  if(size >= FALL_MEM_MAX) return;

  patterns[size] = cur;

  char key[16];
  snprintf(key,sizeof(key),"p%u",size);
  prefs.putBytes(key, &cur, sizeof(FallPattern));

  size++;
  prefs.putUInt("size", size);
}
void FallMemory::setStandBaseline(float ab, float gb){
  standAb = ab;
  standGb = gb;
}

float FallMemory::getStandAb(){
  return standAb;
}

float FallMemory::getStandGb(){
  return standGb;
}

/* ===== clear ===== */
void FallMemory::clear(){
  prefs.clear();
  size = 0;
}

/* ===== debug ===== */
void FallMemory::debugPrint(){
  Serial.println("=== FALL PATTERN MEMORY ===");
  Serial.print("Count: "); Serial.println(size);
  for(uint16_t i=0;i<size;i++){
    Serial.print("#"); Serial.print(i);
    Serial.print(" A(");
    Serial.print(patterns[i].ax,2); Serial.print(",");
    Serial.print(patterns[i].ay,2); Serial.print(",");
    Serial.print(patterns[i].az,2); Serial.print(")");
    Serial.print(" G(");
    Serial.print(patterns[i].gx,2); Serial.print(",");
    Serial.print(patterns[i].gy,2); Serial.print(",");
    Serial.print(patterns[i].gz,2); Serial.println(")");
  }
  Serial.println("===========================");
}
