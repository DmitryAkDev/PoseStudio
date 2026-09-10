/**
 * @file database.h
 * @brief The application's SQLite database: bootstrap, migrations, and the shared connection.
 *
 * One named connection is opened once at startup (see initializeDatabase) and handed to every
 * caller through appDatabase() — never open a second connection per call. The schema lives in
 * resources/database/initialize.sql; additive migrations for existing databases live in
 * initializeDatabase() itself (see database.cpp).
 */

#ifndef DATABASE_H
#define DATABASE_H

#include <QSqlDatabase>

/// The name of the single process-wide QSqlDatabase connection. Only database.cpp should ever
/// need it directly; everything else goes through appDatabase().
inline constexpr const char* kDatabaseConnectionName = "db_conn";

/// The shared, already-open application database connection. Valid after initializeDatabase()
/// has run (main.cpp calls it before any widget is constructed). GUI thread only.
QSqlDatabase appDatabase();

enum class DbInitMode {
    Normal,       ///< Open the existing database, creating it if it doesn't exist yet.
    FactoryReset  ///< Delete the existing database file and rebuild the schema from scratch.
};

/**
 * @brief Opens (or rebuilds) the application's SQLite database and returns the connection.
 * @param mode Normal to just connect, FactoryReset to wipe and rebuild the schema.
 */
QSqlDatabase initializeDatabase(DbInitMode mode);

#endif // DATABASE_H