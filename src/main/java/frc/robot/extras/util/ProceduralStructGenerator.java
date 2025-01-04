package frc.robot.extras.util;

import edu.wpi.first.units.Measure;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.reflect.AnnotatedElement;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Modifier;
import java.lang.reflect.RecordComponent;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.BaseStream;

/** A utility class for procedurally generating {@link Struct}s from records and enums. */
public final class ProceduralStructGenerator {
  private ProceduralStructGenerator() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * A functional interface representing a method that retrives a value from a {@link ByteBuffer}.
   */
  @FunctionalInterface
  private interface Unpacker<T> {
    T unpack(ByteBuffer buffer);
  }

  /** A functional interface representing a method that packs a value into a {@link ByteBuffer}. */
  @FunctionalInterface
  private interface Packer<T> {
    ByteBuffer pack(ByteBuffer buffer, T value);

    static <T> Packer<T> fromStruct(Struct<T> struct) {
      return (buffer, value) -> {
        struct.pack(buffer, value);
        return buffer;
      };
    }
  }

  private record PrimType<T>(String name, int size, Unpacker<T> unpacker, Packer<T> packer) {}

  /** A map of primitive types to their schema types. */
  private static final HashMap<Class<?>, PrimType<?>> primitiveTypeMap = new HashMap<>();

  private static <T> void addPrimType(
      Class<T> boxedClass,
      Class<T> primitiveClass,
      String name,
      int size,
      Unpacker<T> unpacker,
      Packer<T> packer) {
    PrimType<T> primType = new PrimType<>(name, size, unpacker, packer);
    primitiveTypeMap.put(boxedClass, primType);
    primitiveTypeMap.put(primitiveClass, primType);
  }

  // Add primitive types to the map
  static {
    addPrimType(
        Integer.class, int.class, "int32", Integer.BYTES, ByteBuffer::getInt, ByteBuffer::putInt);
    addPrimType(
        Double.class,
        double.class,
        "float64",
        Double.BYTES,
        ByteBuffer::getDouble,
        ByteBuffer::putDouble);
    addPrimType(
        Float.class,
        float.class,
        "float32",
        Float.BYTES,
        ByteBuffer::getFloat,
        ByteBuffer::putFloat);
    addPrimType(
        Boolean.class,
        boolean.class,
        "bool",
        Byte.BYTES,
        buffer -> buffer.get() != 0,
        (buffer, value) -> buffer.put((byte) (value ? 1 : 0)));
    addPrimType(
        Character.class,
        char.class,
        "char",
        Character.BYTES,
        ByteBuffer::getChar,
        ByteBuffer::putChar);
    addPrimType(Byte.class, byte.class, "uint8", Byte.BYTES, ByteBuffer::get, ByteBuffer::put);
    addPrimType(
        Short.class, short.class, "int16", Short.BYTES, ByteBuffer::getShort, ByteBuffer::putShort);
    addPrimType(
        Long.class, long.class, "int64", Long.BYTES, ByteBuffer::getLong, ByteBuffer::putLong);
  }

  /**
   * A map of types to their custom struct schemas.
   *
   * <p>This allows adding custom struct implementations for types that are not supported by
   * default. Think of vendor-specific.
   */
  private static final HashMap<Class<?>, Struct<?>> customStructTypeMap = new HashMap<>();

  /**
   * Add a custom struct to the structifier.
   *
   * @param <T> The type the struct is for.
   * @param clazz The class of the type.
   * @param struct The struct to add.
   * @param override Whether to override an existing struct. An existing struct could mean the type
   *     already has a {@code struct} field and implemnts {@link StructSerializable} or that the
   *     type is already in the custom struct map.
   */
  public static <T> void addCustomStruct(Class<T> clazz, Struct<T> struct, boolean override) {
    if (override) {
      customStructTypeMap.put(clazz, struct);
    } else if (!StructSerializable.class.isAssignableFrom(clazz)) {
      customStructTypeMap.putIfAbsent(clazz, struct);
    }
  }

  /**
   * Returns a {@link Struct} for the given {@link StructSerializable} marked class. Due to the
   * non-contractual nature of the marker this can fail. If the {@code struct} field could not be
   * accessed for any reason, an empty {@link Optional} is returned.
   *
   * @param <T> The type of the class.
   * @param clazz The class object to extract the struct from.
   * @return An optional containing the struct if it could be extracted.
   */
  @SuppressWarnings("unchecked")
  public static <T extends StructSerializable> Optional<Struct<T>> extractClassStruct(
      Class<? extends T> clazz) {
    try {
      var possibleField = Optional.ofNullable(clazz.getDeclaredField("struct"));
      return possibleField.flatMap(
          field -> {
            field.setAccessible(true);
            if (Struct.class.isAssignableFrom(field.getType())) {
              try {
                return Optional.ofNullable((Struct<T>) field.get(null));
              } catch (IllegalAccessException e) {
                return Optional.empty();
              }
            } else {
              return Optional.empty();
            }
          });
    } catch (NoSuchFieldException e) {
      return Optional.empty();
    }
  }

  /**
   * Returns a {@link Struct} for the given class. This does not do compile time checking that the
   * class is a {@link StructSerializable}. Whenever possible it is reccomended to use {@link
   * #extractClassStruct(Class)}.
   *
   * @param clazz The class object to extract the struct from.
   * @return An optional containing the struct if it could be extracted.
   */
  @SuppressWarnings("unchecked")
  public static Optional<Struct<?>> extractClassStructDynamic(Class<?> clazz) {
    if (StructSerializable.class.isAssignableFrom(clazz)) {
      return extractClassStruct((Class<? extends StructSerializable>) clazz).map(struct -> struct);
    } else {
      return Optional.empty();
    }
  }

  @Retention(RetentionPolicy.RUNTIME)
  @Target({ElementType.FIELD, ElementType.RECORD_COMPONENT})
  @Documented
  public @interface IgnoreStructField {}

  @Retention(RetentionPolicy.RUNTIME)
  @Target({ElementType.FIELD, ElementType.RECORD_COMPONENT})
  @Documented
  public @interface FixedSizeArray {
    int size();
  }

  private static OptionalInt arraySize(AnnotatedElement field) {
    return Optional.ofNullable(field.getAnnotation(FixedSizeArray.class))
        .map(FixedSizeArray::size)
        .map(OptionalInt::of)
        .orElse(OptionalInt.empty());
  }

  private static boolean shouldIgnore(AnnotatedElement field) {
    return field.isAnnotationPresent(IgnoreStructField.class);
  }

  private record StructField(
      String name,
      String type,
      int size,
      boolean immutable,
      Set<Struct<?>> structsToLoad,
      Unpacker<?> unpacker,
      Packer<?> packer) {

    public static StructField fromField(Field field) {
      return StructField.fromNameAndClass(
          field.getName(),
          field.getType(),
          arraySize(field),
          Modifier.isFinal(field.getModifiers()));
    }

    public static StructField fromRecordComponent(RecordComponent component) {
      return StructField.fromNameAndClass(
          component.getName(), component.getType(), arraySize(component), true);
    }

    @SuppressWarnings("unchecked")
    public static StructField fromNameAndClass(
        String name, Class<?> clazz, OptionalInt arraySize, boolean isFinal) {
      if (!isFixedSize(clazz, arraySize)) {
        return null;
      }
      if (clazz.isArray() && arraySize.isPresent()) {
        final Class<?> componentType = clazz.getComponentType();
        final int size = arraySize.getAsInt();
        final StructField componentField =
            fromNameAndClass(
                componentType.getSimpleName(), componentType, OptionalInt.empty(), false);
        return new StructField(
            name + "[" + size + "]",
            componentField.type,
            componentField.size * size,
            isFinal,
            componentField.structsToLoad,
            buffer -> {
              Object[] array = new Object[size];
              for (int i = 0; i < size; i++) {
                array[i] = componentField.unpacker.unpack(buffer);
              }
              return array;
            },
            (buffer, value) -> {
              for (Object obj : (Object[]) value) {
                ((Packer<Object>) componentField.packer).pack(buffer, obj);
              }
              return buffer;
            });
      } else if (Measure.class.isAssignableFrom(clazz)) {
        return new StructField(
            name,
            "float64",
            Double.BYTES,
            isFinal,
            Set.of(),
            buffer -> {
              throw new UnsupportedOperationException("Cannot unpack Measure");
            },
            (buffer, value) -> buffer.putDouble(((Measure<?>) value).baseUnitMagnitude()));
      } else if (primitiveTypeMap.containsKey(clazz)) {
        PrimType<?> primType = primitiveTypeMap.get(clazz);
        return new StructField(
            name,
            primType.name,
            primType.size,
            isFinal,
            Set.of(),
            primType.unpacker,
            primType.packer);
      } else {
        Struct<?> struct = null;
        if (customStructTypeMap.containsKey(clazz)) {
          struct = customStructTypeMap.get(clazz);
        } else if (StructSerializable.class.isAssignableFrom(clazz)) {
          struct = extractClassStructDynamic(clazz).orElse(null);
        }
        if (struct == null) {
          return null;
        }
        Set<Struct<?>> structsToLoad = new HashSet<>(Set.of(struct.getNested()));
        structsToLoad.add(struct);
        return new StructField(
            name,
            struct.getTypeName(),
            struct.getSize(),
            struct.isImmutable() && isFinal,
            structsToLoad,
            struct::unpack,
            Packer.fromStruct(struct));
      }
    }
  }

  /**
   * Introspects a class to determine if it's a fixed size.
   *
   * <p>Fixed size means no collections, no strings, no arrays, etc.
   *
   * @param clazz The class to introspect.
   * @return Whether the class is fixed size.
   */
  public static boolean isFixedSize(Class<?> clazz, OptionalInt arraySize) {
    if (clazz.isArray()) {
      if (arraySize.isEmpty()) {
        return false;
      } else {
        Class<?> componentType = clazz.getComponentType();
        return isFixedSize(componentType, OptionalInt.empty());
      }
    } else if (clazz.isRecord()) {
      for (RecordComponent component : clazz.getRecordComponents()) {
        if (!isFixedSize(component.getType(), arraySize(component))) {
          return false;
        }
      }
    } else {
      for (Field field : clazz.getDeclaredFields()) {
        Class<?> fieldClass = field.getType();
        if (field.isEnumConstant() || Modifier.isStatic(field.getModifiers())) {
          continue;
        }
        if (Collection.class.isAssignableFrom(fieldClass)
            || Iterator.class.isAssignableFrom(fieldClass)
            || Iterable.class.isAssignableFrom(fieldClass)
            || BaseStream.class.isAssignableFrom(fieldClass)
            || fieldClass.isArray()
            || fieldClass == String.class
            || fieldClass == Optional.class) {
          return false;
        }
        if (!primitiveTypeMap.containsKey(fieldClass)
            && !isFixedSize(fieldClass, arraySize(field))) {
          return false;
        }
      }
    }
    return true;
  }

  /**
   * Introspects a class to determine if it's interiorly mutable.
   *
   * <p>Interior mutability means that the class has fields that are mutable.
   *
   * @param clazz The class to introspect.
   * @return Whether the class is interiorly mutable.
   */
  public static boolean isInteriorlyMutable(Class<?> clazz) {
    if (clazz.isArray()) {
      return true;
    } else if (clazz.isRecord()) {
      for (RecordComponent component : clazz.getRecordComponents()) {
        if (isInteriorlyMutable(component.getType())) {
          return true;
        }
      }
    } else {
      for (Field field : clazz.getDeclaredFields()) {
        if (field.isEnumConstant() || Modifier.isStatic(field.getModifiers())) {
          continue;
        }
        if (!Modifier.isFinal(field.getModifiers())) {
          return true;
        }
        if (!primitiveTypeMap.containsKey(field.getType())
            && isInteriorlyMutable(field.getType())) {
          return true;
        }
      }
    }
    return false;
  }

  /** A utility for building schema syntax in a procedural manner. */
  @SuppressWarnings("PMD.AvoidStringBufferField")
  public static class SchemaBuilder {
    /** A utility for building enum fields in a procedural manner. */
    public static final class EnumFieldBuilder {
      private final StringBuilder m_builder = new StringBuilder();
      private final String m_fieldName;
      private boolean m_firstVariant = true;

      /**
       * Creates a new enum field builder.
       *
       * @param fieldName The name of the field.
       */
      public EnumFieldBuilder(String fieldName) {
        this.m_fieldName = fieldName;
        m_builder.append("enum {");
      }

      /**
       * Adds a variant to the enum field.
       *
       * @param name The name of the variant.
       * @param value The value of the variant.
       * @return The builder for chaining.
       */
      public EnumFieldBuilder addVariant(String name, int value) {
        if (!m_firstVariant) {
          m_builder.append(',');
        }
        m_firstVariant = false;
        m_builder.append(name).append('=').append(value);
        return this;
      }

      /**
       * Builds the enum field. If this object is being used with {@link SchemaBuilder#addEnumField}
       * then {@link #build()} does not have to be called by the user.
       *
       * @return The built enum field.
       */
      public String build() {
        m_builder.append("} int8 ").append(m_fieldName).append(';');
        return m_builder.toString();
      }
    }

    /** Creates a new schema builder. */
    public SchemaBuilder() {}

    private final StringBuilder m_builder = new StringBuilder();

    /**
     * Adds a field to the schema.
     *
     * @param name The name of the field.
     * @param type The type of the field.
     * @return The builder for chaining.
     */
    public SchemaBuilder addField(StructField field) {
      m_builder.append(field.type).append(' ').append(field.name).append(';');
      return this;
    }

    /**
     * Adds an inline enum field to the schema.
     *
     * @param enumFieldBuilder The builder for the enum field.
     * @return The builder for chaining.
     */
    public SchemaBuilder addEnumField(EnumFieldBuilder enumFieldBuilder) {
      m_builder.append(enumFieldBuilder.build());
      return this;
    }

    /**
     * Builds the schema.
     *
     * @return The built schema.
     */
    public String build() {
      return m_builder.toString();
    }
  }

  public static <T> Struct<T> noopStruct(Class<T> cls) {
    return new Struct<>() {
      @Override
      public Class<T> getTypeClass() {
        return cls;
      }

      @Override
      public String getTypeName() {
        return cls.getSimpleName();
      }

      @Override
      public String getSchema() {
        return "";
      }

      @Override
      public int getSize() {
        return 0;
      }

      @Override
      public void pack(ByteBuffer buffer, T value) {}

      @Override
      public T unpack(ByteBuffer buffer) {
        return null;
      }
    };
  }

  private abstract static class ProcStruct<T> implements Struct<T> {
    protected final Class<T> typeClass;
    protected final List<StructField> fields;
    private final String schema;

    // stored values so we never recompute them
    private final int size;
    private final boolean isImmutable;
    private final Struct<?>[] nested;

    public ProcStruct(Class<T> typeClass, List<StructField> fields, String schema) {
      this.typeClass = typeClass;
      this.fields = fields;
      this.schema = schema;

      this.size = fields.stream().mapToInt(StructField::size).sum();
      this.isImmutable = fields.stream().allMatch(StructField::immutable);
      this.nested =
          fields.stream()
              .map(StructField::structsToLoad)
              .flatMap(Collection::stream)
              .toArray(Struct<?>[]::new);

      ProceduralStructGenerator.customStructTypeMap.put(typeClass, this);
    }

    @Override
    public Class<T> getTypeClass() {
      return typeClass;
    }

    @Override
    public String getTypeName() {
      return typeClass.getSimpleName();
    }

    @Override
    public String getSchema() {
      return schema;
    }

    @Override
    public int getSize() {
      return size;
    }

    @Override
    public boolean isCloneable() {
      return Cloneable.class.isAssignableFrom(typeClass);
    }

    @Override
    @SuppressWarnings("unchecked")
    public T clone(T obj) throws CloneNotSupportedException {
      if (isCloneable()) {
        try {
          return (T) typeClass.getMethod("clone").invoke(obj);
        } catch (IllegalAccessException | InvocationTargetException | NoSuchMethodException e) {
          throw new CloneNotSupportedException();
        }
      } else {
        throw new CloneNotSupportedException();
      }
    }

    @Override
    public boolean isImmutable() {
      return isImmutable;
    }

    @Override
    public Struct<?>[] getNested() {
      return nested;
    }

    @Override
    public String toString() {
      return this.getTypeName() + "<" + this.getSize() + ">" + " {" + this.schema + "}";
    }
  }

  /**
   * Generates a {@link Struct} for the given {@link Record} class. If a {@link Struct} cannot be
   * generated from the {@link Record}, the errors encountered will be printed and a no-op {@link
   * Struct} will be returned.
   *
   * @param <R> The type of the record.
   * @param recordClass The class of the record.
   * @return The generated struct.
   */
  @SuppressWarnings({"unchecked", "PMD.AvoidAccessibilityAlteration"})
  public static <R extends Record> Struct<R> genRecord(final Class<R> recordClass) {
    final RecordComponent[] components = recordClass.getRecordComponents();
    final SchemaBuilder schemaBuilder = new SchemaBuilder();
    final ArrayList<StructField> fields = new ArrayList<>();

    for (final RecordComponent component : components) {
      if (shouldIgnore(component)) {
        continue;
      }
      component.getAccessor().setAccessible(true);
      fields.add(StructField.fromRecordComponent(component));
    }

    if (fields.stream().anyMatch(f -> f == null)) {
      return noopStruct(recordClass);
    }
    fields.forEach(schemaBuilder::addField);

    return new ProcStruct<>(recordClass, fields, schemaBuilder.build()) {
      @Override
      public void pack(ByteBuffer buffer, R value) {
        boolean failed = false;
        int startingPosition = buffer.position();
        for (int i = 0; i < components.length; i++) {
          Packer<Object> packer = (Packer<Object>) fields.get(i).packer();
          try {
            Object componentValue = components[i].getAccessor().invoke(value);
            if (componentValue == null) {
              throw new IllegalArgumentException("Component is null");
            }
            packer.pack(buffer, componentValue);
          } catch (IllegalAccessException
              | IllegalArgumentException
              | InvocationTargetException e) {
            failed = true;
            break;
          }
        }
        if (failed) {
          buffer.put(startingPosition, new byte[this.getSize()]);
        }
      }

      @Override
      public R unpack(ByteBuffer buffer) {
        try {
          Object[] args = new Object[components.length];
          Class<?>[] argTypes = new Class<?>[components.length];
          for (int i = 0; i < components.length; i++) {
            args[i] = fields.get(i).unpacker().unpack(buffer);
            argTypes[i] = components[i].getType();
          }
          return recordClass.getConstructor(argTypes).newInstance(args);
        } catch (InstantiationException
            | IllegalAccessException
            | InvocationTargetException
            | NoSuchMethodException
            | SecurityException e) {
          System.err.println(
              "Could not unpack record: "
                  + recordClass.getSimpleName()
                  + "\n    "
                  + e.getMessage());
          return null;
        }
      }
    };
  }

  /**
   * Generates a {@link Struct} for the given {@link Enum} class. If a {@link Struct} cannot be
   * generated from the {@link Enum}, the errors encountered will be printed and a no-op {@link
   * Struct} will be returned.
   *
   * @param <E> The type of the enum.
   * @param enumClass The class of the enum.
   * @return The generated struct.
   */
  @SuppressWarnings({"unchecked", "PMD.AvoidAccessibilityAlteration"})
  public static <E extends Enum<E>> Struct<E> genEnum(Class<E> enumClass) {
    final E[] enumVariants = enumClass.getEnumConstants();
    final Field[] allEnumFields = enumClass.getDeclaredFields();
    final SchemaBuilder schemaBuilder = new SchemaBuilder();
    final SchemaBuilder.EnumFieldBuilder enumFieldBuilder =
        new SchemaBuilder.EnumFieldBuilder("variant");
    final HashMap<Integer, E> enumMap = new HashMap<>();
    final ArrayList<StructField> fields = new ArrayList<>();

    if (enumVariants == null || enumVariants.length == 0) {
      return noopStruct(enumClass);
    }

    for (final E constant : enumVariants) {
      final String name = constant.name();
      final int ordinal = constant.ordinal();

      enumFieldBuilder.addVariant(name, ordinal);
      enumMap.put(ordinal, constant);
    }
    schemaBuilder.addEnumField(enumFieldBuilder);
    fields.add(
        new StructField(
            "variant",
            "int8",
            1,
            true,
            Set.of(),
            ByteBuffer::get,
            (buffer, value) -> buffer.put((byte) ((Enum<?>) value).ordinal())));

    final List<Field> enumFields =
        List.of(allEnumFields).stream()
            .filter(
                f ->
                    !f.isEnumConstant() && !Modifier.isStatic(f.getModifiers()) && !shouldIgnore(f))
            .toList();

    for (final Field field : enumFields) {
      field.setAccessible(true);
      fields.add(StructField.fromField(field));
    }
    if (fields.stream().anyMatch(f -> f == null)) {
      return noopStruct(enumClass);
    }
    for (int i = 1; i < fields.size(); i++) {
      // do this to skip the variant field
      schemaBuilder.addField(fields.get(i));
    }

    return new ProcStruct<>(enumClass, fields, schemaBuilder.build()) {
      @Override
      public void pack(ByteBuffer buffer, E value) {
        boolean failed = false;
        int startingPosition = buffer.position();
        buffer.put((byte) value.ordinal());
        for (int i = 0; i < enumFields.size(); i++) {
          Packer<Object> packer = (Packer<Object>) fields.get(i + 1).packer();
          Field field = enumFields.get(i);
          try {
            Object fieldValue = field.get(value);
            if (fieldValue == null) {
              throw new IllegalArgumentException("Field is null");
            }
            packer.pack(buffer, fieldValue);
          } catch (IllegalArgumentException | IllegalAccessException e) {
            System.err.println(
                "Could not pack enum field: "
                    + enumClass.getSimpleName()
                    + "#"
                    + field.getName()
                    + "\n    "
                    + e.getMessage());
            failed = true;
            break;
          }
        }
        if (failed) {
          buffer.put(startingPosition, new byte[this.getSize()]);
        }
      }

      final byte[] m_spongeBuffer = new byte[this.getSize() - 1];

      @Override
      public E unpack(ByteBuffer buffer) {
        int ordinal = buffer.get();
        buffer.get(m_spongeBuffer);
        return enumMap.getOrDefault(ordinal, null);
      }

      public boolean isCloneable() {
        return true;
      }
      ;

      public E clone(E obj) throws CloneNotSupportedException {
        return obj;
      }
      ;
    };
  }

  /**
   * Generates a {@link Struct} for the given {@link Object} class. If a {@link Struct} cannot be
   * generated from the {@link Object}, the errors encountered will be printed and a no-op {@link
   * Struct} will be returned.
   *
   * @param <O> The type of the object.
   * @param objectClass The class of the object.
   * @param objectSupplier A supplier for the object.
   * @return The generated struct.
   */
  @SuppressWarnings({"unchecked", "PMD.AvoidAccessibilityAlteration"})
  public static <O> Struct<O> genObject(Class<O> objectClass, Supplier<O> objectSupplier) {
    final SchemaBuilder schemaBuilder = new SchemaBuilder();
    final Field[] allFields =
        List.of(objectClass.getDeclaredFields()).stream()
            .filter(f -> !shouldIgnore(f) && !Modifier.isStatic(f.getModifiers()))
            .toArray(Field[]::new);
    final ArrayList<StructField> fields = new ArrayList<>(allFields.length);

    for (final Field field : allFields) {
      field.setAccessible(true);
      fields.add(StructField.fromField(field));
    }

    if (fields.stream().anyMatch(f -> f == null)) {
      return noopStruct(objectClass);
    }
    fields.forEach(schemaBuilder::addField);

    return new ProcStruct<>(objectClass, fields, schemaBuilder.build()) {
      @Override
      public void pack(ByteBuffer buffer, O value) {
        boolean failed = false;
        int startingPosition = buffer.position();
        for (int i = 0; i < allFields.length; i++) {
          Packer<Object> packer = (Packer<Object>) fields.get(i).packer();
          try {
            Object fieldValue = allFields[i].get(value);
            if (fieldValue == null) {
              throw new IllegalArgumentException("Field is null");
            }
            packer.pack(buffer, fieldValue);
          } catch (IllegalArgumentException | IllegalAccessException e) {
            System.err.println(
                "Could not pack object field: "
                    + objectClass.getSimpleName()
                    + "#"
                    + allFields[i].getName()
                    + "\n    "
                    + e.getMessage());
            failed = true;
            break;
          }
        }
        if (failed) {
          buffer.put(startingPosition, new byte[this.getSize()]);
        }
      }

      @Override
      public O unpack(ByteBuffer buffer) {
        try {
          O obj = objectSupplier.get();
          for (int i = 0; i < allFields.length; i++) {
            Object fieldValue = fields.get(i).unpacker().unpack(buffer);
            allFields[i].set(obj, fieldValue);
          }
          return obj;
        } catch (IllegalArgumentException | IllegalAccessException e) {
          System.err.println(
              "Could not unpack object: "
                  + objectClass.getSimpleName()
                  + "\n    "
                  + e.getMessage());
          return null;
        }
      }
    };
  }
}
