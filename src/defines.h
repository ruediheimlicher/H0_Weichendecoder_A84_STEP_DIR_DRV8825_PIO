//
//  defines.h
//  H0_Decoder
//
//  Created by Ruedi Heimlicher on 11.09.2020.
//

#ifndef defines_h
#define defines_h

#define LOOPLED PA7
#define LOOPLED_DDR DDRA
#define LOOPLED_PORT PORTA

#define SLEEP_PIN PB0


#define OSZI_DDR  DDRA
#define OSZI_PORT PORTA
#define OSZIA     5
#define OSZIB     6

#define OSZIALO OSZI_PORT &= ~(1<<OSZIA)
#define OSZIAHI OSZI_PORT |= (1<<OSZIA)
#define OSZIATOG OSZI_PORT ^= (1<<OSZIA)

#define OSZIBLO OSZI_PORT &= ~(1<<OSZIB)
#define OSZIBHI OSZI_PORT |= (1<<OSZIB)
#define OSZIBTOG OSZI_PORT ^= (1<<OSZIB)



#endif