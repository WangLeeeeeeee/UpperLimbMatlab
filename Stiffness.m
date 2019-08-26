% ��������������λ������ؽڸն�
% ��ؽ�ֻ��һ�����ɶȣ����һ��ֵ����ؽ��������ɶȣ��ؽڸնȾ���Ϊ3*3������ؽڸնȾ���ļ�
function [K_ELBOW_Value,K_SH_Value] = Stiffness(q,tension)

[~,n] = size(q);
len = n;
K_ELBOW_Value = zeros(len,1);
K_SH_Value = zeros(len,1);
% �ؽڸն�
for k=1:len
    [JF_SH,JF_EL] = TensiontoTorque(q(:,k));
    KCable_1 = nsu(tension(1,k));
    KCable_2 = nsu(tension(2,k));
    KCable_3 = nsu(tension(3,k));
    KCable_4 = nsu(tension(4,k));
    KCable_5 = nsu(tension(5,k));
    KCable_6 = nsu(tension(6,k));   
    KCable_sh = diag([KCable_1,KCable_2,KCable_3,KCable_4]);
    KCable_el = diag([KCable_5,KCable_6]);
    K_SH = JF_SH*KCable_sh*(JF_SH)';
    K_SH_Value(k) = trace(K_SH);
    K_ELBOW_Value(k) = JF_EL(3,:)*KCable_el*(JF_EL(3,:))';
end

end