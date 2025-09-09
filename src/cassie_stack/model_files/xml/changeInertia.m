ratio = 2.5;


mlStruct = readstruct('cassieRigid.xml');


%base
rawmass = mlStruct.worldbody.body.inertial.massAttribute;
mlStruct.worldbody.body.inertial.massAttribute = rawmass;

base_com = str2num(mlStruct.worldbody.body.inertial.posAttribute);
base_com(3) = base_com(3);
mlStruct.worldbody.body.inertial.posAttribute = num2str(base_com);
mlStruct.worldbody.body.inertial.fullinertiaAttribute = num2str( str2num(mlStruct.worldbody.body.inertial.fullinertiaAttribute)*mlStruct.worldbody.body.inertial.massAttribute/rawmass   );

%leg

for i = 1:2
    %achilles-rod, knee
    for j = 1:2
        mlStruct.worldbody.body.body(i).body.body.body(j).inertial = changeInertial(mlStruct.worldbody.body.body(i).body.body.body(j).inertial,ratio);
    end
    
    %knee-spring, shin
    for j = 1:2
        mlStruct.worldbody.body.body(i).body.body.body(2).body(j).inertial = changeInertial(mlStruct.worldbody.body.body(i).body.body.body(2).body(j).inertial,ratio);
    end
    
    %tarsus
    mlStruct.worldbody.body.body(i).body.body.body(2).body(2).body.inertial = changeInertial(mlStruct.worldbody.body.body(i).body.body.body(2).body(2).body.inertial,ratio);
    
    %heel-spring,foot-crank,foot
    for m = 1:3
        mlStruct.worldbody.body.body(i).body.body.body(2).body(2).body.body(m).inertial =  changeInertial(mlStruct.worldbody.body.body(i).body.body.body(2).body(2).body.body(m).inertial,ratio);
    end
    %plantar-rod
    mlStruct.worldbody.body.body(i).body.body.body(2).body(2).body.body(2).body.inertial = changeInertial(mlStruct.worldbody.body.body(i).body.body.body(2).body(2).body.body(2).body.inertial,ratio);
end


writestruct(mlStruct,'exported.xml','StructNodeName','mujoco')

%!!!IMPORTANT!!! HAVE TO change sensor order MANUALLY


%%
function outstruct = changeInertial(instruct,ratio)
tmp = instruct;
tmp.massAttribute = ratio*tmp.massAttribute;
tmp.fullinertiaAttribute = num2str(ratio*str2num(tmp.fullinertiaAttribute));
outstruct = tmp;
end