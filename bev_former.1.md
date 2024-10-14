

# bev_former

- 有6个编码层，主体框架与传统transformer一样
- 
## Transformer
QKV
q问的是在不在前面，k是回答，它俩相乘计算的是相关性，用相关性乘以v，得到一个delta变化的向量，基础向量加这个向量就会得到一个新的向量，这个向量会丰富原词语，在前面丰富，因为q问的是在不在前面，但是这个在不在前面是怎么体现的，应该是在回传更新的时候体现，或者是在不在前面这句话有一个向量encoding?那又是怎么体现的，又或者是在q*k后将后面的都设置为0？
q和k的维度会比较低，所以乘以的w维度与v的不同？q和k相当于作了降维，v没有降维，得到的就是一个与其他embedding在同等维度的向量，虽然是delta，但它要用于加法运算，也不能降维

“汇聚”通常指的是将多个向量或信息整合成一个单一的表示

k、v相当于已知的x_ i，y_i，q是未知的x，用已知的去求f(x)，
**Transformer 主要用于分类任务和生成任务，而不是回归任务**
### **Transformer 架构如何适应不同任务**

Transformer 的核心架构，包括**多头自注意力机制**和**前馈神经网络**，本质上是通用的。根据不同的任务，它的输出层和损失函数会有所不同：

-   **分类任务**：
    
    -   输出层通常是一个全连接层，后接 Softmax 激活函数，用于将输出转化为类别概率分布。
    -   损失函数通常使用交叉熵损失函数（Cross-Entropy Loss），用来衡量预测的类别分布与真实类别之间的差异。
-   **回归任务**：
    
    -   输出层是一个线性层，不使用激活函数，直接输出连续值。
    -   损失函数通常是均方误差（Mean Squared Error, MSE）或均方根误差（Root Mean Squared Error, RMSE），用来衡量预测值与真实值之间的差异。
    
<!--stackedit_data:
eyJoaXN0b3J5IjpbLTYzMTYyMTczMywtMjA0ODQyNzA3LDE2Nj
A5NzQxNzIsLTExOTY1OTMzMjddfQ==
-->