# -*- coding: utf-8 -*-
"""
Created on Thu Apr 29 15:23:06 2021

@author: Natha
"""
import tensorflow as tf

foo = tf.constant([0.5, -1.1, 4.1, -2.1, 0.2], dtype = tf.float32)
val = tf.keras.activations.relu(foo).numpy()

print(val)