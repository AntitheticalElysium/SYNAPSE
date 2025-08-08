import tensorflow as tf

def create_model(input_shape):
    model = tf.keras.models.Sequential([
        # Normalize pixel values
        tf.keras.layers.Rescaling(1./255, input_shape=input_shape),
        
        tf.keras.layers.Conv2D(16, (3, 3), activation='relu'),
        tf.keras.layers.MaxPooling2D(2, 2),
        tf.keras.layers.Conv2D(32, (3, 3), activation='relu'),
        tf.keras.layers.MaxPooling2D(2, 2),
        tf.keras.layers.Flatten(),
        tf.keras.layers.Dense(64, activation='relu'),
        # Output layer: 1 neuron with sigmoid for binary classification (novel/boring)
        tf.keras.layers.Dense(1, activation='sigmoid')
    ])
    
    model.compile(optimizer='adam',
                  loss=tf.keras.losses.BinaryCrossentropy(),
                  metrics=['accuracy'])
    
    return model