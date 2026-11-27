# Jamiro

Synthé polyphonique pour MPC standalone, par Yann.

**Jamiro est une recréation du synthé intégré au clavier Novation X-Station** (le moteur
« KS », partagé avec les KS4, KS5, KS Rack et K-Station). Il reprend la même architecture, les
mêmes menus et les mêmes valeurs 0..127 / -64..+63 que la machine : les patchs X-Station se
transposent paramètre par paramètre, et le convertisseur lit directement les banques `.syx`
de l'X-Station, comme la mise à jour « 100 New Sounds » et la banque Jamiroquai publiées par
Novation pour ce clavier.

Le nom « Jamiro » est un clin d'œil à cette banque. Jamiro n'est ni affilié à Novation ou à
Jamiroquai, ni approuvé par eux. X-Station et Novation sont des marques de Focusrite/Novation.
Le code est entièrement original, et aucun son ni échantillon de Novation n'y est copié.

Il faut MPC OS 3.10.1 ou plus récent. Activez les plugins externes dans
Preferences → Enhancements (Beta) → Enable Enhancements and External Plugin Support.

## Ce qu'il fait

- 8 voix, 3 oscillateurs par voix, avec bruit et ring-mod 1×2. Unisson jusqu'à 8 voix par note.
- Modes Mono, Mono AG (glide automatique en legato), Poly 1 et Poly 2. Portamento.
- 17 formes d'onde : sinus, triangle, dent de scie (avec « double saw » via le PW), pulse à
  largeur variable, 4 bruits, et 9 formes « échantillonnées » (orgue, clavecin, piano électrique,
  slap bass, Rhodes, tine, Wurly, Clavinet, basse analogique). Ces 9 formes sont recréées par
  synthèse additive : ce ne sont pas les échantillons d'origine.
- Sync osc 1 > 2, FM osc 2 > 3 avec son enveloppe AD.
- Filtre passe-bas, passe-bande ou passe-haut, 12 ou 24 dB, avec overdrive, Q normalise,
  suivi de clavier et modulation par enveloppe, LFO2, vélocité, molette et aftertouch.
- Enveloppes Amp et Mod (ADSR + vélocité). 2 LFO (10 formes, delay, sync au tempo, keysync, unipolaire).
- Arpégiateur interne calé sur le tempo de la MPC : Up, Down, Up-Down 1/2, Played, Random, Chord,
  1 à 4 octaves, gate, latch.
- Effets : distorsion, EQ tilt, chorus/phaser, delay stéréo (sync possible), reverb (6 tailles), pan + auto-pan.

## Programmes

Le plugin ne contient **aucun son de Novation**. Il propose `Init Program` et 6 sons originaux
(« Jamiro Saw Lead », « Sub Bass », « Warm Pad », « Funk Clav », « Arp Pulse », « FM Bell »).

**Pour retrouver les sons de l'X-Station**, copie des banques `.syx` dans le dossier
`Content/Banks/` du plugin sur ta carte SD :

```
VST2 Plugins/Yann - VST - Jamiro/Content/Banks/1 New Sounds.syx
VST2 Plugins/Yann - VST - Jamiro/Content/Banks/2 Jamiroquai.syx
```

Au démarrage, Jamiro convertit tous les programmes qu'il y trouve, dans l'ordre alphabétique
des fichiers : le 1er fichier donne les programmes 100 à 199, le 2e les programmes 200 à 299,
etc. Il accepte :
- les dumps de banque ou de programme de l'X-Station ;
- les dumps de programme ou de paire de programmes des KS4, KS5, KS Rack et K-Station.

Novation a publié pour l'X-Station la banque « 100 New Sounds » et une banque Jamiroquai, en
téléchargement pour les possesseurs du clavier. Récupère-les chez Novation : elles ne sont pas
distribuées avec Jamiro. Pour les sons connus, un ajustement de volume est appliqué
(`src/jamiro_trims.h`, qui ne contient que des sommes de contrôle et des nombres) pour que tout
sorte à un niveau comparable.

Ce sont des conversions, pas des copies exactes de la machine. Les courbes (temps d'enveloppe,
vitesses de LFO, profondeurs) ont été estimées, et quelques réglages X-Station ne sont pas
reproduits : patterns rythmiques de l'arpégiateur, LFO « one-shot », formes de LFO à motifs
(remplacées par S/H quantifié), ratio du delay, modulation de l'EQ.

Les sons d'arpèges et de percussions sont en général plus discrets. Quelques effets spéciaux
évoluent très lentement (LFO synchronisés sur 8 bars), par exemple « 188 Scratch Perverts ».

## Pages à l'écran (6)

| Page | Contrôles |
|---|---|
| OSC 1-2 | Onde, octave, demi-ton, cents, PW, PW←LFO2, pitch←LFO1, pitch←Mod env (osc 1 puis osc 2) |
| OSC 3 | Idem pour l'osc 3 ; FM 2>3 (niveau, enveloppe, attaque, decay) ; PW←Mod env ×3 ; Sync |
| MIXER | Niveaux osc 1/2/3, bruit, ring ; Mix level, Volume, Pan ; mode de voix, unisson, détune, portamento ; molette→vibrato, molette→filtre, aftertouch→filtre, bend |
| FILT/ENV | Cutoff, résonance, overdrive, key track ; env, LFO2, type, pente ; enveloppe Mod ; enveloppe Amp |
| LFO/ARP | LFO1 et LFO2 (onde, vitesse, delay, sync) ; arpégiateur ; vélocité Amp et Mod |
| FX | Mix chorus, envoi delay, envoi reverb, distorsion ; chorus ; delay ; reverb ; EQ |

Ces réglages n'ont pas de place à l'écran, mais ils sont sauvegardés, chargés par les programmes
et automatisables : VCO Drift, Q Normalise, Filt Velo, MW Filt LFO, AT Vibrato, AutoPan Dep/Rate,
LFO KeySync/Unipol, Arp Tempo (utilisé seulement si la MPC ne donne pas de tempo), et les
modulations de niveau du mixer (LFO1>O1, LFO2>O1, LFO2>O2, FMEnv>O3).

## CPU

Sur PC, le plugin prend 0,5 à 3 % selon le programme. Les sons les plus gourmands combinent
unisson, 3 oscillateurs, filtre 24 dB et chorus. Sur la MPC, compte plusieurs fois plus.
Évite d'empiler beaucoup d'instances de pads en unisson 8 sur une MPC 32 bits.

## Construire

Depuis le dossier du MPC Plugin SDK :

```
python tools/build.py plugins/Jamiro
python plugins/Jamiro/tools/make_skin.py      # seulement après avoir modifié ui.json
python tools/build.py plugins/Jamiro
python tools/package.py plugins/Jamiro
```

## Fichiers

- `src/Jamiro.cpp` : paramètres, voix, arpégiateur, chaîne d'effets, programmes originaux.
- `src/jamiro_dsp.h` : courbes, oscillateurs, tables d'ondes, LFO, effets.
- `src/ks_import.h` : lecture des banques `.syx` KS/X-Station dans `Content/Banks/`.
- `src/jamiro_trims.h` : ajustements de volume (sommes de contrôle + dB).
- `ui.json` : pages de l'écran tactile ; `skins/` : skin généré par `tools/make_skin.py`.
- `art/jamiro_bg_public.png` : fond de l'écran (création originale).

Licence : MIT, code original. La reverb est un réseau Schroeder/Moorer classique (domaine public).
Le suréchantillonnage 4x (filtres demi-bande IIR polyphase) suit la méthode de la bibliothèque
HIIR de Laurent de Soras (licence WTFPL) ; les coefficients ont été calculés pour Jamiro.

## Version 1.1

- Overdrive et Distortion calculés en suréchantillonnage 4x : plus d'aliasing métallique dans les
  aigus quand on pousse le drive (basses heavy, electro, reese / DNB).
- Compensation de niveau automatique : le volume reste le même du réglage 0 au réglage 127.
- 1.1.1 : plus de grésillement quand on tourne Overdrive ou Distortion (bouton lissé, l'effet
  arrive en fondu sur les premiers crans) ; l'Overdrive à 127 n'est plus beaucoup moins fort
  qu'à 0 ; une nouvelle note ne démarre plus trop bas.
- 1.1.2 : Distortion refaite comme une pédale de basse : même grain du début à la fin de la
  note (elle ne dépend plus du volume), 20 % de son clair gardé pour le corps, drive moins
  extrême et filtre de tonalité (18 kHz → 6 kHz) contre le sifflement dans les aigus.
- 1.1.3 : plus de notes bloquées en Mono / Mono AG.
- 1.1.4 : l'Overdrive est de nouveau AVANT le filtre, comme sur la X-Station et en 1.0.6.
  De la 1.0.7 à la 1.1.3, il était après le filtre : les basses filtrées devenaient des ondes
  carrées avec un grésillement aigu permanent (jusqu'à 10 kHz) sur les notes graves, même
  filtre fermé (ex. 119 « Old Funk Bass », et les ~100 patchs des banques qui utilisent
  l'Overdrive). Gardés : suréchantillonnage 4x, bouton lissé, volume compensé (compensation
  rapide au début de la note, lente ensuite, pour ne pas onduler sur les graves). Nouveau
  filtre anti-DC à 5 Hz en sortie : les ondes pulse très étroites créaient un fort décalage
  continu (« poc » en début/fin de note, marge perdue). Volume de 14 patchs réajusté pour
  rester comme en 1.1.3.
- Mêmes paramètres qu'en 1.0 : banques et projets se rechargent à l'identique.
- CPU : un peu plus quand Overdrive ou Distortion sont au-dessus de 0 (rien de changé à 0).

