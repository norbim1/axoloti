<patch-1.0>
   <comment type="comment" name="for use as subpatch" x="19" y="24"/>
   <comment type="comment" name="parent patch must contain a table named t, size 8192" x="20" y="60"/>
   <obj type="randtrig" sha="7c693e3fcb8abe7dc3908628ef0eb911a4a19ce1" name="randtrig_1" x="20" y="100">
      <params/>
      <attribs/>
   </obj>
   <obj type="*c" sha="1ea155bb99343babad87e3ff0de80e6bf568e8da" name="lspread" x="100" y="100">
      <params>
         <frac32.u.map name="amp" onParent="true" value="4.0"/>
      </params>
      <attribs/>
   </obj>
   <obj type="envdlinmx" sha="a2e1da37932bdfc8056cd08cca74d2ebc6735f40" name="length" x="200" y="100">
      <params>
         <frac32.s.map name="d" onParent="true" value="5.0"/>
      </params>
      <attribs/>
   </obj>
   <obj type="&gt;" sha="66caccb0a5493a2894a4643178419d30d067add1" name="&gt;_1" x="320" y="100">
      <params/>
      <attribs/>
   </obj>
   <obj type="inv" sha="77e1f7e6c23d3e299a726997943d432a8cde98bd" name="inv_1" x="400" y="100">
      <params/>
      <attribs/>
   </obj>
   <obj type="tabplay2~" sha="98247b4a866cd88ae46eeb0743bbb1a9fff17514" name="tabplay2~_1" x="720" y="220">
      <params/>
      <attribs>
         <objref attributeName="table" obj="../t"/>
      </attribs>
   </obj>
   <obj type="window" sha="ff29ab0721db1b1238076400832e919d860fc38f" name="window_1" x="440" y="280">
      <params/>
      <attribs/>
   </obj>
   <obj type="vca~" sha="6bbeaeb94e74091879965461ad0cb043f2e7f6cf" name="vca~_1" x="940" y="300">
      <params/>
      <attribs/>
   </obj>
   <obj type="div2" sha="5df68ad33aa1633cb7cb1724fcd41eee28932582" name="div2_1" x="1020" y="300">
      <params/>
      <attribs/>
   </obj>
   <obj type="outlet~" sha="72226293648dde4dd4739bc1b8bc46a6bf660595" name="outlet~_1" x="1160" y="300">
      <params/>
      <attribs/>
   </obj>
   <obj type="*c" sha="1ea155bb99343babad87e3ff0de80e6bf568e8da" name="fb" x="480" y="320">
      <params>
         <frac32.u.map name="amp" onParent="true" value="4.0"/>
      </params>
      <attribs/>
   </obj>
   <obj type="interp~" sha="5a9175b8d44d830756d1599a86b4a6a49813a19b" name="interp~_2" x="540" y="320">
      <params/>
      <attribs/>
   </obj>
   <obj type="xfade" sha="e301a25211e0b6899b8b54c25e7ac5d9f404eda0" name="xfade_1" x="620" y="320">
      <params/>
      <attribs/>
   </obj>
   <obj type="tabrecord2~" sha="a81485c5ad3662cfb26feb163c52775cbd64fe38" name="tabrecord2~_1" x="720" y="320">
      <params/>
      <attribs>
         <objref attributeName="table" obj="../t"/>
      </attribs>
   </obj>
   <obj type="inlet~" sha="2944bdbaeb2a8a42d5a97163275d052f75668a86" name="inlet~_1" x="360" y="340">
      <params/>
      <attribs/>
   </obj>
   <nets>
      <net>
         <source name="length env"/>
         <dest name="window_1 phase"/>
         <dest name="&gt;_1 in1"/>
      </net>
      <net>
         <source name="inv_1 o"/>
         <dest name="length trig"/>
         <dest name="randtrig_1 trig"/>
      </net>
      <net>
         <source name="randtrig_1 rand"/>
         <dest name="lspread in"/>
      </net>
      <net>
         <source name="lspread out"/>
         <dest name="length dm"/>
      </net>
      <net>
         <source name="window_1 win"/>
         <dest name="fb in"/>
         <dest name="vca~_1 v"/>
      </net>
      <net>
         <source name="tabplay2~_1 wave"/>
         <dest name="xfade_1 i1"/>
         <dest name="vca~_1 a"/>
      </net>
      <net>
         <source name="&gt;_1 out"/>
         <dest name="tabplay2~_1 en"/>
         <dest name="tabrecord2~_1 en"/>
         <dest name="inv_1 i"/>
      </net>
      <net>
         <source name="xfade_1 o"/>
         <dest name="tabrecord2~_1 wave"/>
      </net>
      <net>
         <source name="inlet~_1 inlet"/>
         <dest name="xfade_1 i2"/>
      </net>
      <net>
         <source name="div2_1 out"/>
         <dest name="outlet~_1 outlet"/>
      </net>
      <net>
         <source name="fb out"/>
         <dest name="interp~_2 i"/>
      </net>
      <net>
         <source name="interp~_2 o"/>
         <dest name="xfade_1 c"/>
      </net>
      <net>
         <source name="vca~_1 o"/>
         <dest name="div2_1 in"/>
      </net>
   </nets>
   <settings>
      <subpatchmode>polyphonic</subpatchmode>
      <MidiChannel>1</MidiChannel>
      <NPresets>0</NPresets>
      <NPresetEntries>0</NPresetEntries>
      <NModulationSources>0</NModulationSources>
      <NModulationTargetsPerSource>0</NModulationTargetsPerSource>
   </settings>
   <notes><![CDATA[]]></notes>
</patch-1.0>